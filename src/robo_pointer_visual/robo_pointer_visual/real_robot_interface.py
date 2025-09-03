import numpy as np
import math
import time
import json
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
import traceback
from ament_index_python.packages import get_package_share_directory
from typing import Tuple, Optional, Dict, Any, List
import enum
from pathlib import Path

# Utilisation directe de FeetechMotorsBus
# Importation directe du SDK bas niveau
try:
    import scservo_sdk as scs
    from scservo_sdk import GroupSyncRead, GroupSyncWrite
except ImportError:
    print("ERREUR CRITIQUE: Le paquet scservo_sdk n'est pas installé ou incomplet.")
    scs = None

# --- Constantes et Définitions ---
NODE_NAME = 'real_robot_interface'
PACKAGE_NAME = 'robo_pointer_visual'
DEFAULT_CALIBRATION_FILENAME = 'manual_calibration_follower.json'

LEADER_ARM_PORT = "/dev/ttyACM0"
LEADER_ARM_MOTORS = {
    "shoulder_pan": [1, "sts3215"],
    "shoulder_lift": [2, "sts3215"],
    "elbow_flex": [3, "sts3215"],
    "wrist_flex": [4, "sts3215"],
    "wrist_roll": [5, "sts3215"],
    "gripper": [6, "sts3215"],
}
MOTOR_NAMES_ORDER = list(LEADER_ARM_MOTORS.keys())
MODEL_RESOLUTION = {"sts3215": 4096}
HALF_TURN_DEGREE = 180.0

SCS_CONTROL_TABLE = {
    "Torque_Enable": (40, 1),
    "Goal_Position": (42, 2),
    "Present_Position": (56, 2),
    "Status_Return_Level": (16, 1),
}

MOTORS_TO_READ = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

class CalibrationMode(enum.Enum):
    DEGREE = 0
    LINEAR = 1

# Les fonctions de calibration restent inchangées
def apply_calibration(raw_values: np.ndarray, motor_names: list[str], calibration_data: dict, motors_def: dict):
    if calibration_data is None: return np.full(len(motor_names), np.nan, dtype=np.float32)
    calibrated_values = np.full(len(motor_names), np.nan, dtype=np.float32)
    for i, name in enumerate(motor_names):
        if name not in calibration_data["motor_names"]: continue
        try:
            calib_idx = calibration_data["motor_names"].index(name)
            calib_mode_str = calibration_data["calib_mode"][calib_idx]
            calib_mode = CalibrationMode[calib_mode_str.upper()]
            drive_mode = calibration_data["drive_mode"][calib_idx]
            homing_offset = float(calibration_data["homing_offset"][calib_idx])
            _, model = motors_def[name]
            resolution = float(MODEL_RESOLUTION[model])
            current_raw_value = float(raw_values[i])
            steps_per_180_deg = resolution / 2.0
            if calib_mode == CalibrationMode.DEGREE:
                if abs(steps_per_180_deg) < 1e-9: continue
                raw_zero = -homing_offset
                delta_raw = current_raw_value - raw_zero
                delta_normalized = delta_raw
                if delta_raw > steps_per_180_deg: delta_normalized = delta_raw - resolution
                elif delta_raw < -steps_per_180_deg: delta_normalized = delta_raw + resolution
                delta_steps_calibrated = delta_normalized * (1.0 if drive_mode == 0 else -1.0)
                calibrated_values[i] = delta_steps_calibrated / steps_per_180_deg * HALF_TURN_DEGREE
            elif calib_mode == CalibrationMode.LINEAR:
                start_pos = float(calibration_data["start_pos"][calib_idx])
                end_pos = float(calibration_data["end_pos"][calib_idx])
                if abs(end_pos - start_pos) < 1e-6:
                    calibrated_values[i] = 50.0
                else:
                    percentage = (current_raw_value - start_pos) / (end_pos - start_pos) * 100.0
                    calibrated_values[i] = np.clip(percentage, -10.0, 110.0)
        except (KeyError, IndexError, ValueError):
            pass
    return calibrated_values

def revert_calibration(target_values_calibrated: np.ndarray, motor_names: list[str], calibration_data: dict, motors_def: dict):
    if calibration_data is None: return None
    target_steps = np.zeros(len(motor_names), dtype=np.int32)
    for i, name in enumerate(motor_names):
        motor_model_tuple = motors_def.get(name, ["", "sts3215"])
        default_step = MODEL_RESOLUTION.get(motor_model_tuple[1], 4096) // 2
        if name not in calibration_data["motor_names"]:
            target_steps[i] = default_step; continue
        try:
            calib_idx = calibration_data["motor_names"].index(name)
            calib_mode_str = calibration_data["calib_mode"][calib_idx]
            calib_mode = CalibrationMode[calib_mode_str.upper()]
            hw_start_pos = float(calibration_data["start_pos"][calib_idx])
            hw_end_pos = float(calibration_data["end_pos"][calib_idx])
            hw_min_step = min(hw_start_pos, hw_end_pos)
            hw_max_step = max(hw_start_pos, hw_end_pos)
            drive_mode = calibration_data["drive_mode"][calib_idx]
            homing_offset = float(calibration_data["homing_offset"][calib_idx])
            _, model = motor_model_tuple
            resolution = float(MODEL_RESOLUTION[model])
            current_target_calibrated = float(target_values_calibrated[i])
            target_step_raw = float(default_step)
            if calib_mode == CalibrationMode.DEGREE:
                steps_per_180_deg = resolution / 2.0
                if abs(steps_per_180_deg) < 1e-9: continue
                step_centered = current_target_calibrated / HALF_TURN_DEGREE * steps_per_180_deg
                if drive_mode == 1: step_centered *= -1.0
                target_step_raw = step_centered - homing_offset
            elif calib_mode == CalibrationMode.LINEAR:
                start_pos_json = hw_min_step
                end_pos_json = hw_max_step
                if abs(end_pos_json - start_pos_json) < 1e-6:
                    target_step_raw = start_pos_json
                else:
                    target_percentage_clipped = np.clip(current_target_calibrated, 0.0, 100.0)
                    target_step_raw = (target_percentage_clipped / 100.0) * (end_pos_json - start_pos_json) + start_pos_json
            final_step_value_clipped = int(round(np.clip(target_step_raw, hw_min_step, hw_max_step)))
            target_steps[i] = final_step_value_clipped
        except (KeyError, IndexError, ValueError):
            target_steps[i] = default_step
    return target_steps

class RealRobotInterfaceNode(Node):
    """
    Nœud ROS 2 pour l'interface directe avec les servomoteurs du bras robotique.
    - Lit la position des moteurs en temps réel.
    - Applique une calibration pour convertir les valeurs brutes en degrés.
    - Publie l'état des articulations (/joint_states).
    - Reçoit des angles cibles (/target_joint_angles), les convertit en pas moteur et les envoie.
    """
    def __init__(self):
        super().__init__(NODE_NAME)
        self.get_logger().info(f"Node '{NODE_NAME}' starting...")
        
        if scs is None:
             self.get_logger().fatal("scservo_sdk not found. Aborting.")
             raise ImportError("scservo_sdk is required.")

        # Initialiser les attributs pour un cleanup plus sûr
        self.leader_bus_instance = None
        self.port_handler = None
        self.packet_handler = None
        self.group_writer = None
        self.group_reader = None
        self.reader_motors_ok = False

        try:
            # Paramètres
            self.declare_parameter('leader_arm_port', LEADER_ARM_PORT)
            self.declare_parameter('read_frequency_hz', 20.0)

            self._load_calibration_file()
            self._connect_motor_bus()
            self._initialize_group_sync_handlers()
            self._initialize_motors_torque()
            read_frequency = self.get_parameter('read_frequency_hz').get_parameter_value().double_value
            
            self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
            self.target_angles_subscription = self.create_subscription(
                JointState, '/target_joint_angles', self.target_angles_callback, 10
            )
            self.read_publish_timer = self.create_timer(1.0 / read_frequency, self.read_and_publish_states)
            
            self.get_logger().info('Node initialized successfully. Ready to execute commands.')

        except Exception as e_init:
            self.get_logger().fatal(f"CRITICAL ERROR during node initialization: {e_init}")
            traceback.print_exc()
            self.cleanup_resources()
            raise RuntimeError("Node initialization failed") from e_init

    def _load_calibration_file(self):
        package_share_directory_str = get_package_share_directory(PACKAGE_NAME)
        package_share_path = Path(package_share_directory_str)
        calibration_file_path = package_share_path / 'config' / DEFAULT_CALIBRATION_FILENAME
        
        with open(calibration_file_path, 'r') as f:
            self.calibration_data = json.load(f)
        self.get_logger().info(f"Calibration data loaded from: {calibration_file_path}")

    def _connect_motor_bus(self):
        # Import retardé pour éviter un échec d'import au chargement des tests
        try:
            from lerobot.common.robot_devices.motors.feetech import (
                FeetechMotorsBus, FeetechMotorsBusConfig,
            )
        except Exception as e:
            raise ImportError("lerobot feetech drivers not available. Ensure the 'lerobot' env is active.") from e

        port_param = self.get_parameter('leader_arm_port').get_parameter_value().string_value or LEADER_ARM_PORT
        leader_config = FeetechMotorsBusConfig(port=port_param, motors=LEADER_ARM_MOTORS)
        self.leader_bus_instance = FeetechMotorsBus(leader_config)
        self.leader_bus_instance.connect()
        self.port_handler = self.leader_bus_instance.port_handler
        self.packet_handler = self.leader_bus_instance.packet_handler
        self.get_logger().info("Successfully connected to motor bus.")

    def _initialize_group_sync_handlers(self):
        addr_goal_pos, len_goal_pos = SCS_CONTROL_TABLE["Goal_Position"]
        self.group_writer = GroupSyncWrite(self.port_handler, self.packet_handler, addr_goal_pos, len_goal_pos)
        
        addr_present_pos, len_present_pos = SCS_CONTROL_TABLE["Present_Position"]
        self.group_reader = GroupSyncRead(self.port_handler, self.packet_handler, addr_present_pos, len_present_pos)
        
        self.group_reader.clearParam()
        self.reader_motors_ok = True
        for name in MOTORS_TO_READ:
            motor_id = LEADER_ARM_MOTORS[name][0]
            if not self.group_reader.addParam(motor_id):
                self.get_logger().error(f"GroupSyncRead addParam failed for {name}(ID:{motor_id})")
                self.reader_motors_ok = False
        self.get_logger().info("Group sync handlers initialized.")

    def _initialize_motors_torque(self):
        self.get_logger().info("Enabling torque for all motors...")
        addr_torque = SCS_CONTROL_TABLE["Torque_Enable"][0]
        for name in MOTOR_NAMES_ORDER:
            motor_id = LEADER_ARM_MOTORS[name][0]
            scs_comm_result, _ = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, addr_torque, 1)
            if scs_comm_result != scs.COMM_SUCCESS:
                 self.get_logger().warn(f"Torque enable for {name}(ID:{motor_id}) potentially failed.")
            time.sleep(0.02)
        self.get_logger().info("Motor torque enabled.")
        
    def read_and_publish_states(self):
        if not self.reader_motors_ok: return
        
        comm_result = self.group_reader.txRxPacket()
        if comm_result != scs.COMM_SUCCESS:
            self.get_logger().warn("Failed to read motor states.", throttle_duration_sec=2)
            return

        raw_positions = []
        all_data_available = True
        for name in MOTORS_TO_READ:
            motor_id = LEADER_ARM_MOTORS[name][0]
            addr_present_pos, len_present_pos = SCS_CONTROL_TABLE["Present_Position"]
            if self.group_reader.isAvailable(motor_id, addr_present_pos, len_present_pos):
                value = self.group_reader.getData(motor_id, addr_present_pos, len_present_pos)
                raw_positions.append(value)
            else:
                all_data_available = False; break
        
        if not all_data_available:
            self.get_logger().warn("Incomplete data from motor read.", throttle_duration_sec=2)
            return
            
        calibrated_angles_deg = apply_calibration(np.array(raw_positions), MOTORS_TO_READ, self.calibration_data, LEADER_ARM_MOTORS)
        
        if np.isnan(calibrated_angles_deg).any():
            self.get_logger().warn("Calibration resulted in NaN values.", throttle_duration_sec=2)
            return
            
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = MOTORS_TO_READ
        joint_state_msg.position = [math.radians(angle) for angle in calibrated_angles_deg]
        self.joint_state_publisher.publish(joint_state_msg)

    def target_angles_callback(self, msg: JointState):
        self.get_logger().debug(f"Received target angles for joints: {msg.name}")
        target_motor_names = msg.name
        target_angles_rad = msg.position
        target_angles_deg = [math.degrees(angle) for angle in target_angles_rad]

        target_steps = revert_calibration(
            np.array(target_angles_deg), target_motor_names, self.calibration_data, LEADER_ARM_MOTORS
        )
        if target_steps is None:
            self.get_logger().error("Failed to convert target angles to steps (revert_calibration failed).")
            return

        self.group_writer.clearParam()
        success_add = True
        for i, name in enumerate(target_motor_names):
            if name in LEADER_ARM_MOTORS:
                motor_id = LEADER_ARM_MOTORS[name][0]
                step_value = int(target_steps[i])
                param = [scs.SCS_LOBYTE(step_value), scs.SCS_HIBYTE(step_value)]
                if not self.group_writer.addParam(motor_id, param):
                    self.get_logger().error(f"GroupSyncWrite addParam failed for {name}(ID {motor_id})")
                    success_add = False
        
        if not success_add: return
        
        comm_result = self.group_writer.txPacket()
        if comm_result != scs.COMM_SUCCESS:
            self.get_logger().error("GroupSyncWrite txPacket error.", throttle_duration_sec=1.0)
            
    def cleanup_resources(self):
        self.get_logger().info("Attempting resource cleanup...")
        if hasattr(self, 'leader_bus_instance') and self.leader_bus_instance and self.leader_bus_instance.is_connected:
            self.leader_bus_instance.disconnect()
            self.get_logger().info("Leader bus disconnected.")

    def destroy_node(self):
        self.get_logger().info(f"Shutting down {NODE_NAME} node...")
        self.cleanup_resources()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = RealRobotInterfaceNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        if executor: executor.shutdown()
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
