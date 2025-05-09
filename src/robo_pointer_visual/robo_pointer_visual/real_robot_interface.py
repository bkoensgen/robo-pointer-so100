# Fichier: ~/ros2_ws/src/robo_pointer_visual/robo_pointer_visual/real_robot_interface.py
# Version: ID2_ANGULAR_COMPENSATION_V1

import numpy as np
import math
import time
import json
from pathlib import Path
import enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import traceback
from ament_index_python.packages import get_package_share_directory

# Utilisation directe de FeetechMotorsBus (pour config et connexion bas niveau)
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus, FeetechMotorsBusConfig
# Importation directe du SDK bas niveau
try:
    import scservo_sdk as scs
    from scservo_sdk import GroupSyncRead
    from scservo_sdk import GroupSyncWrite
except ImportError:
    print("ERREUR CRITIQUE: Le paquet scservo_sdk n'est pas installé ou incomplet.")
    print("Veuillez l'installer via 'pip install -e \".[feetech]\"' dans le dossier lerobot.")
    scs = None
    GroupSyncRead = None
    GroupSyncWrite = None

# Import local de la cinématique
from .kinematics import calculate_fk_wrist, calculate_ik

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
MOTORS_TO_COMMAND = ["shoulder_pan", "shoulder_lift", "elbow_flex"]
INITIAL_HOLD_POSE_DEG = np.array([0.0, 90.0, 90.0], dtype=np.float32)
INITIAL_WRIST_POS_DEG = 0.0

class CalibrationMode(enum.Enum):
    DEGREE = 0
    LINEAR = 1

# --- Fonctions de Calibration ---
def apply_calibration(raw_values: np.ndarray, motor_names: list[str], calibration_data: dict, motors_def: dict):
    """
    Convertit les steps bruts lus des moteurs (int32) en valeurs calibrées 
    (degrés pour les joints rotatifs, pourcentage pour les linéaires) en float32.
    Gère le wrap-around pour les moteurs rotatifs.
    """
    if calibration_data is None:
        print("[ERROR] apply_calibration: calibration_data is None!")
        return np.full(len(motor_names), np.nan, dtype=np.float32)

    calibrated_values = np.full(len(motor_names), np.nan, dtype=np.float32)

    for i, name in enumerate(motor_names):
        if name not in calibration_data["motor_names"]:
            continue # Motor not in calibration file, skip

        try:
            calib_idx = calibration_data["motor_names"].index(name)
            # Basic checks for data integrity
            if not (calib_idx < len(calibration_data.get("drive_mode", [])) and \
                    calib_idx < len(calibration_data.get("homing_offset", [])) and \
                    calib_idx < len(calibration_data.get("calib_mode", []))):
                print(f"[ERROR apply_calib] Calibration data lists mismatch for motor '{name}'.")
                continue

            calib_mode_str = calibration_data["calib_mode"][calib_idx]
            try: calib_mode = CalibrationMode[calib_mode_str]
            except KeyError:
                print(f"[ERROR apply_calib] Invalid calib_mode '{calib_mode_str}' for motor '{name}'.")
                continue

            drive_mode = calibration_data["drive_mode"][calib_idx]
            homing_offset = float(calibration_data["homing_offset"][calib_idx])
            _, model = motors_def[name]
            resolution = float(MODEL_RESOLUTION[model])
            current_raw_value = float(raw_values[i])
            steps_per_180_deg = resolution / 2.0

            if calib_mode == CalibrationMode.DEGREE:
                if abs(steps_per_180_deg) < 1e-9: continue # Invalid resolution

                raw_zero = -homing_offset # Homing offset is defined relative to the "zero step"
                delta_raw = current_raw_value - raw_zero
                # Handle wrap-around: if delta is > half turn, assume it wrapped
                delta_normalized = delta_raw
                if delta_raw > steps_per_180_deg: delta_normalized = delta_raw - resolution
                elif delta_raw < -steps_per_180_deg: delta_normalized = delta_raw + resolution
                
                delta_steps_calibrated = delta_normalized
                if drive_mode == 1: delta_steps_calibrated *= -1.0 # Invert if drive mode is 1

                calibrated_degrees = delta_steps_calibrated / steps_per_180_deg * HALF_TURN_DEGREE
                calibrated_values[i] = calibrated_degrees

            elif calib_mode == CalibrationMode.LINEAR:
                 if not (calib_idx < len(calibration_data.get("start_pos", [])) and \
                         calib_idx < len(calibration_data.get("end_pos", []))):
                      continue # Missing start/end_pos for linear motor
                 start_pos = float(calibration_data["start_pos"][calib_idx])
                 end_pos = float(calibration_data["end_pos"][calib_idx])
                 if abs(end_pos - start_pos) < 1e-6: # Avoid division by zero
                      calibrated_values[i] = 0.0 if current_raw_value == start_pos else 50.0
                 else:
                      percentage = (current_raw_value - start_pos) / (end_pos - start_pos) * 100.0
                      calibrated_values[i] = np.clip(percentage, -10.0, 110.0) # Allow slight over/undershoot
        except Exception as e:
            print(f"[ERROR apply_calib] Unexpected error processing motor '{name}': {e}")
            traceback.print_exc()
    return calibrated_values

def revert_calibration(target_values_calibrated: np.ndarray, motor_names: list[str], calibration_data: dict, motors_def: dict):
    """
    Convertit les valeurs cibles calibrées (degrés/pourcentage) en steps bruts (int32) pour l'envoi aux moteurs.
    Les steps finaux sont clippés aux limites start_pos/end_pos définies dans le fichier JSON de calibration,
    puis aux limites absolues du moteur (0 à résolution-1).
    """
    if calibration_data is None:
         print("[ERROR revert_calib] calibration_data is None!")
         return None

    target_steps = np.zeros(len(motor_names), dtype=np.int32)

    for i, name in enumerate(motor_names):
        motor_model_tuple = motors_def.get(name, ["", "sts3215"]) # Default to sts3215 if name not found
        default_step = MODEL_RESOLUTION.get(motor_model_tuple[1], 4096) // 2

        if name not in calibration_data["motor_names"]:
            target_steps[i] = default_step
            continue

        try:
            calib_idx = calibration_data["motor_names"].index(name)
            if not (calib_idx < len(calibration_data.get("drive_mode", [])) and \
                    calib_idx < len(calibration_data.get("homing_offset", [])) and \
                    calib_idx < len(calibration_data.get("calib_mode", []))):
                print(f"[ERROR revert_calib] Calibration data lists mismatch for motor '{name}'.")
                target_steps[i] = default_step
                continue

            calib_mode_str = calibration_data["calib_mode"][calib_idx]
            try: calib_mode = CalibrationMode[calib_mode_str]
            except KeyError:
                print(f"[ERROR revert_calib] Invalid calib_mode '{calib_mode_str}' for motor '{name}'.")
                target_steps[i] = default_step
                continue

            # Determine hardware step limits from JSON if available
            apply_hw_clip_from_json = False
            hw_min_step_json = 0.0
            hw_max_step_json = float(MODEL_RESOLUTION.get(motor_model_tuple[1], 4096) -1)
            if calib_idx < len(calibration_data.get("start_pos", [])) and \
               calib_idx < len(calibration_data.get("end_pos", [])):
                hw_start_pos = float(calibration_data["start_pos"][calib_idx])
                hw_end_pos = float(calibration_data["end_pos"][calib_idx])
                hw_min_step_json = min(hw_start_pos, hw_end_pos)
                hw_max_step_json = max(hw_start_pos, hw_end_pos)
                apply_hw_clip_from_json = True
            
            drive_mode = calibration_data["drive_mode"][calib_idx]
            homing_offset = float(calibration_data["homing_offset"][calib_idx])
            _, model = motor_model_tuple
            resolution = float(MODEL_RESOLUTION[model])
            current_target_calibrated = float(target_values_calibrated[i])
            target_step_raw = float(default_step) # Fallback

            if calib_mode == CalibrationMode.DEGREE:
                steps_per_180_deg = resolution / 2.0
                if abs(steps_per_180_deg) < 1e-9: continue
                step_centered = current_target_calibrated / HALF_TURN_DEGREE * steps_per_180_deg
                if drive_mode == 1: step_centered *= -1.0
                target_step_raw = step_centered - homing_offset # homing_offset is defined as "value to subtract"
            elif calib_mode == CalibrationMode.LINEAR:
                 if not apply_hw_clip_from_json: # Linear mode requires start/end pos from JSON
                      target_steps[i] = default_step
                      continue
                 start_pos_json = hw_min_step_json
                 end_pos_json = hw_max_step_json
                 if abs(end_pos_json - start_pos_json) < 1e-6:
                     target_step_raw = start_pos_json
                 else:
                     target_percentage_clipped = np.clip(current_target_calibrated, 0.0, 100.0)
                     target_step_raw = (target_percentage_clipped / 100.0) * (end_pos_json - start_pos_json) + start_pos_json

            # Apply clipping: first to JSON limits (if any), then to motor's absolute limits
            final_step_value_clipped_json = target_step_raw
            if apply_hw_clip_from_json:
                final_step_value_clipped_json = np.clip(target_step_raw, hw_min_step_json, hw_max_step_json)
            
            # Final clip to absolute motor range [0, resolution-1]
            final_step_value_abs_clipped = int(round(np.clip(final_step_value_clipped_json, 0, resolution - 1)))
            target_steps[i] = final_step_value_abs_clipped

        except Exception as e:
            print(f"[ERROR revert_calib] Unexpected error processing motor '{name}': {e}")
            traceback.print_exc()
            target_steps[i] = default_step
    return target_steps

class RealRobotInterfaceNode(Node):
    """
    ROS 2 Node to interface with the real robot arm (SO-100 or similar Feetech-based).
    It subscribes to control signals, reads motor states, calculates target motor commands
    using Inverse Kinematics (IK) and Proportional-Integral (PI) control,
    and sends these commands to the motors.
    """
    def __init__(self):
        super().__init__(NODE_NAME)
        self.get_logger().info(f"{NODE_NAME} node starting... (ID2_ANGULAR_COMPENSATION_V1 Version)")
        self.leader_bus_instance = None
        self.port_handler = None
        self.packet_handler = None
        self.calibration_data = None
        self.group_writer = None
        self.group_reader = None
        self.control_signal_subscription = None
        self.reader_motors_ok = False
        self.last_valid_target_angles = None
        self.last_sent_target_steps = None
        self.last_known_raw_positions = None
        self.last_read_time = self.get_clock().now()

        if scs is None or GroupSyncRead is None or GroupSyncWrite is None:
             self.get_logger().fatal("scservo_sdk or required Group classes not found. Aborting.")
             raise ImportError("scservo_sdk is required but not installed or incomplete.")

        try:
            # 1. Load Manual Calibration File
            try:
                package_share_directory = get_package_share_directory(PACKAGE_NAME)
                calibration_file_path = Path(package_share_directory) / 'config' / DEFAULT_CALIBRATION_FILENAME
            except Exception as e:
                self.get_logger().fatal(f"Could not find package share directory '{PACKAGE_NAME}' or construct path: {e}")
                raise RuntimeError("Failed to get calibration file path") from e
            
            if not calibration_file_path.exists():
                self.get_logger().error(f"MANUAL Calibration file not found at: {calibration_file_path}.")
                raise FileNotFoundError(f"MANUAL Calibration file not found: {calibration_file_path}")
            with open(calibration_file_path, 'r') as f:
                self.calibration_data = json.load(f)
            self.get_logger().info(f"MANUAL Calibration data loaded from: {calibration_file_path}")
            # Basic validation of calibration file content
            expected_keys = ["motor_names", "homing_offset", "drive_mode", "calib_mode", "start_pos", "end_pos"]
            if not all(k in self.calibration_data for k in expected_keys):
                 self.get_logger().error("MANUAL Calibration file missing required keys!")
                 raise ValueError("Invalid MANUAL calibration file structure.")
            if self.calibration_data.get("motor_names") != MOTOR_NAMES_ORDER:
                 self.get_logger().error("Motor order mismatch in MANUAL calibration file!")
                 raise ValueError("Motor order mismatch in MANUAL calibration file.")

            # 2. Connect to Motor Bus
            leader_config = FeetechMotorsBusConfig(port=LEADER_ARM_PORT, motors=LEADER_ARM_MOTORS)
            self.leader_bus_instance = FeetechMotorsBus(leader_config)
            self.get_logger().info(f"Connecting to bus on {LEADER_ARM_PORT}...")
            self.leader_bus_instance.connect()
            self.port_handler = self.leader_bus_instance.port_handler
            self.packet_handler = self.leader_bus_instance.packet_handler
            if self.port_handler is None or self.packet_handler is None:
                 raise RuntimeError("Failed to get valid port/packet handlers.")
            self.get_logger().info("Successfully connected to motor bus.")

            # 3. Configure Status Return Level = 0 for controlled motors
            addr_ret_level = SCS_CONTROL_TABLE["Status_Return_Level"][0]
            self.get_logger().info(f"Setting Status Return Level to 0 for motors: {MOTORS_TO_COMMAND}...")
            for name in MOTORS_TO_COMMAND:
                 if name not in LEADER_ARM_MOTORS: continue
                 motor_id = LEADER_ARM_MOTORS[name][0]
                 scs_comm_result, scs_error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, addr_ret_level, 0)
                 if scs_comm_result != scs.COMM_SUCCESS or scs_error != 0:
                      self.get_logger().warn(f"Failed to set Status Return Level for {name}({motor_id}). Res:{scs_comm_result}, Err:{scs_error}")
                 time.sleep(0.02) # Short delay between configurations

            # 4. Initialize GroupSyncWrite & GroupSyncRead
            addr_goal_pos, len_goal_pos = SCS_CONTROL_TABLE["Goal_Position"]
            self.group_writer = GroupSyncWrite(self.port_handler, self.packet_handler, addr_goal_pos, len_goal_pos)
            if self.group_writer is None: raise RuntimeError("Failed to initialize GroupSyncWrite.")
            self.get_logger().info("GroupSyncWrite handler initialized.")
            
            addr_present_pos, len_present_pos = SCS_CONTROL_TABLE["Present_Position"]
            self.group_reader = GroupSyncRead(self.port_handler, self.packet_handler, addr_present_pos, len_present_pos)
            if self.group_reader is None: raise RuntimeError("Failed to initialize GroupSyncRead.")
            self.get_logger().info("GroupSyncRead handler initialized.")
            
            self.reader_motors_ok = True
            self.group_reader.clearParam()
            for name in MOTORS_TO_COMMAND: # Only add motors we intend to read
                if name not in LEADER_ARM_MOTORS: continue
                motor_id = LEADER_ARM_MOTORS[name][0]
                if not self.group_reader.addParam(motor_id):
                    self.get_logger().error(f"Initial GroupSyncRead addParam failed for {name}({motor_id})")
                    self.reader_motors_ok = False
            if not self.reader_motors_ok:
                 self.get_logger().error("Could not add all target motors to GroupSyncRead handler.")

            # 5. Enable Torque & Set Initial Pose/Hold Target
            motors_to_init_torque = MOTORS_TO_COMMAND + ["wrist_flex"] # Include wrist for initial pose
            self.get_logger().info(f"Enabling torque for {motors_to_init_torque}...")
            addr_torque = SCS_CONTROL_TABLE["Torque_Enable"][0]
            addr_goal_pos_init = SCS_CONTROL_TABLE["Goal_Position"][0] # For individual initial pos
            for name in motors_to_init_torque:
                if name not in LEADER_ARM_MOTORS: continue
                motor_id = LEADER_ARM_MOTORS[name][0]
                # Attempt to enable torque
                scs_comm_result, scs_error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, addr_torque, 1)
                if scs_comm_result != scs.COMM_SUCCESS or (scs_error != 0 and scs_error != 32): # Err 32 can mean "already enabled"
                     self.get_logger().warn(f"Torque enable for {name}({motor_id}) potentially failed. Res:{scs_comm_result}, Err:{scs_error}")
                time.sleep(0.05)
                # Set initial position for wrist_flex only, others will be set by hold logic
                if name == "wrist_flex":
                    target_steps_wrist_array = revert_calibration(np.array([INITIAL_WRIST_POS_DEG]), [name], self.calibration_data, LEADER_ARM_MOTORS)
                    if target_steps_wrist_array is not None:
                        target_steps_wrist = target_steps_wrist_array.item()
                        self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, addr_goal_pos_init, target_steps_wrist)
                        time.sleep(0.5) # Allow wrist to reach initial pos
            
            # Initialize hold target based on INITIAL_HOLD_POSE_DEG
            self.get_logger().info(f"Calculating initial hold target pose: {INITIAL_HOLD_POSE_DEG} deg")
            initial_hold_steps = revert_calibration(
                INITIAL_HOLD_POSE_DEG, MOTORS_TO_COMMAND, self.calibration_data, LEADER_ARM_MOTORS
            )
            if initial_hold_steps is not None:
                self.last_valid_target_angles = INITIAL_HOLD_POSE_DEG.copy()
                self.last_sent_target_steps = initial_hold_steps
                self.get_logger().info(f"Initial hold target set -> Angles: {np.round(self.last_valid_target_angles,1)}, Steps: {self.last_sent_target_steps}")
            else:
                self.get_logger().error("Failed to calculate initial hold steps. Hold logic might start passively.")
            self.get_logger().info("Motor init and initial pose setup complete.")

            # --- 6. ROS Parameters ---
            # General Control Parameters
            self.declare_parameter('stop_threshold', 0.01) # Threshold for control signal magnitude to consider as "stop"
            self.declare_parameter('read_frequency_hz', 20.0) # Target frequency for reading motor states

            # Pan Control Parameters
            self.declare_parameter('pan_increment_scale', 0.1) # Proportional gain for pan control
            self.declare_parameter('pan_angle_min', -110.0)
            self.declare_parameter('pan_angle_max', 110.0)

            # Cartesian Y Target Limits (for IK input)
            self.declare_parameter('target_yw_min', -0.02) # Min Y cartesian target for IK
            self.declare_parameter('target_yw_max', 0.52)  # Max Y cartesian target for IK (previously 0.26)

            # Software Joint Limits (for IK output clipping)
            self.declare_parameter('lift_min', 0.0)   # Min angle for shoulder_lift (ID2)
            self.declare_parameter('lift_max', 130.0)  # Max angle for shoulder_lift (ID2)
            self.declare_parameter('elbow_min', 0.0)  # Min angle for elbow_flex (ID3)
            self.declare_parameter('elbow_max', 110.0) # Max angle for elbow_flex (ID3)
            
            # PI Control for Global Vertical (Y Cartesian) Movement
            self.declare_parameter('kp_vertical_gain', -0.00027) # Proportional gain for Y cartesian error
            self.declare_parameter('ki_vertical_gain', -0.0001)  # Integral gain for Y cartesian error
            self.declare_parameter('integral_max_output', 0.05)  # Anti-windup: Max output from Y integral term
            self.declare_parameter('integral_min_output', -0.05) # Anti-windup: Min output from Y integral term

            # PI Control for ID2 (Shoulder/Lift) Angular Compensation
            self.declare_parameter('kp_angular_comp_id2', 0.008)   # Proportional gain for ID2 angular error
            self.declare_parameter('ki_angular_comp_id2', 0.00025)  # Integral gain for ID2 angular error
            self.declare_parameter('integral_angular_comp_id2_max_output_adj', 3.0) # Anti-windup: Max angle adjustment (deg)
            self.declare_parameter('integral_angular_comp_id2_min_output_adj', -3.0)# Anti-windup: Min angle adjustment (deg)

            # Retrieve parameter values
            self.stop_threshold = self.get_parameter('stop_threshold').get_parameter_value().double_value
            read_frequency = self.get_parameter('read_frequency_hz').get_parameter_value().double_value
            self.read_period = 1.0 / read_frequency if read_frequency > 0 else 0.0

            self.pan_increment_scale = self.get_parameter('pan_increment_scale').get_parameter_value().double_value
            self.pan_min = self.get_parameter('pan_angle_min').get_parameter_value().double_value
            self.pan_max = self.get_parameter('pan_angle_max').get_parameter_value().double_value

            self.Y_MIN_LIMIT = self.get_parameter('target_yw_min').get_parameter_value().double_value
            self.Y_MAX_LIMIT = self.get_parameter('target_yw_max').get_parameter_value().double_value
            self.lift_min = self.get_parameter('lift_min').get_parameter_value().double_value
            self.lift_max = self.get_parameter('lift_max').get_parameter_value().double_value
            self.elbow_min = self.get_parameter('elbow_min').get_parameter_value().double_value
            self.elbow_max = self.get_parameter('elbow_max').get_parameter_value().double_value

            self.Kp_y_cartesian = self.get_parameter('kp_vertical_gain').get_parameter_value().double_value
            self.Ki_y_cartesian = self.get_parameter('ki_vertical_gain').get_parameter_value().double_value
            self.integral_y_cartesian_max_output_val = self.get_parameter('integral_max_output').get_parameter_value().double_value
            self.integral_y_cartesian_min_output_val = self.get_parameter('integral_min_output').get_parameter_value().double_value
            
            self.Kp_angular_comp_ID2 = self.get_parameter('kp_angular_comp_id2').get_parameter_value().double_value
            self.Ki_angular_comp_ID2 = self.get_parameter('ki_angular_comp_id2').get_parameter_value().double_value
            self.integral_angular_comp_ID2_max_output_adj_val = self.get_parameter('integral_angular_comp_id2_max_output_adj').get_parameter_value().double_value
            self.integral_angular_comp_ID2_min_output_adj_val = self.get_parameter('integral_angular_comp_id2_min_output_adj').get_parameter_value().double_value
            
            # Initialize PI controller state variables
            self.integral_accumulator_y_cartesian = 0.0
            self.last_y_cartesian_control_time = self.get_clock().now()
            self.integral_accumulator_angle_id2 = 0.0
            self.last_comp_angle_id2_time = self.get_clock().now()

            # Log parameters
            self.get_logger().info(f"Pan Scale: {self.pan_increment_scale:.4f}, Limits: [{self.pan_min:.1f}, {self.pan_max:.1f}] deg")
            self.get_logger().info(f"Vertical Cartesian Gains: Kp_y={self.Kp_y_cartesian:.5f}, Ki_y={self.Ki_y_cartesian:.5f}")
            self.get_logger().info(f"Vertical Cartesian Integral Output Limits: [{self.integral_y_cartesian_min_output_val:.3f}, {self.integral_y_cartesian_max_output_val:.3f}]")
            self.get_logger().info(f"YW Target Limits (Cartesian for IK): [{self.Y_MIN_LIMIT:.3f}, {self.Y_MAX_LIMIT:.3f}] m")
            self.get_logger().info(f"ID2 Ang. Comp. Gains: Kp_comp={self.Kp_angular_comp_ID2:.4f}, Ki_comp={self.Ki_angular_comp_ID2:.4f}")
            self.get_logger().info(f"ID2 Ang. Comp. Integral Adj Limits: [{self.integral_angular_comp_ID2_min_output_adj_val:.2f}, {self.integral_angular_comp_ID2_max_output_adj_val:.2f}] deg")
            self.get_logger().info(f"Software Joint Limits: Lift=[{self.lift_min:.1f}, {self.lift_max:.1f}], Elbow=[{self.elbow_min:.1f}, {self.elbow_max:.1f}] deg")
            self.get_logger().info(f"Stop Threshold: {self.stop_threshold:.3f}, Read Frequency: {read_frequency:.1f} Hz")

            # Subscriber for control signals
            self.control_signal_subscription = self.create_subscription(
                Vector3, '/robot_control_signal', self.control_signal_callback, 10
            )
            self.get_logger().info('Subscribed to /robot_control_signal')

        except Exception as e_init:
            self.get_logger().fatal(f"CRITICAL ERROR during node initialization: {e_init}")
            traceback.print_exc()
            self.cleanup_resources() # Attempt cleanup before re-raising
            raise RuntimeError("Node initialization failed") from e_init

    def cleanup_resources(self):
        self.get_logger().info("Attempting resource cleanup...")
        if self.leader_bus_instance is not None and getattr(self.leader_bus_instance, 'is_connected', False):
            try:
                self.get_logger().info("Disconnecting leader bus instance...")
                self.leader_bus_instance.disconnect()
                self.get_logger().info("Leader bus disconnected.")
            except Exception as e_disconnect:
                self.get_logger().error(f"Error during leader bus disconnect: {e_disconnect}")
        elif self.port_handler is not None and hasattr(self.port_handler, 'closePort'):
             try:
                  self.get_logger().warn("Leader bus object missing or not connected, attempting direct port close.")
                  self.port_handler.closePort()
                  self.get_logger().info("Port handler closed directly.")
             except Exception as e_close:
                  self.get_logger().error(f"Error closing port handler directly: {e_close}")
        else:
             self.get_logger().info("No active bus/port connection found to clean up.")
        self.port_handler = None # Ensure handlers are None after cleanup
        self.packet_handler = None
        self.group_writer = None
        self.group_reader = None

    def send_goal_positions(self, names: list[str], steps: np.ndarray) -> bool:
        """Sends goal positions to specified motors using GroupSyncWrite."""
        if self.group_writer is None: 
            self.get_logger().error("GroupSyncWrite handler not initialized.", throttle_duration_sec=5)
            return False
        
        # Ensure steps are within the absolute motor limits [0, 4095]
        steps_final_clipped = np.clip(steps, 0, 4095).astype(np.int32)
        if not np.array_equal(steps, steps_final_clipped):
             # Log if any step was outside the absolute [0,4095] range before this final clip
             diff_indices = np.where(steps != steps_final_clipped)[0]
             for idx in diff_indices:
                  self.get_logger().error(
                      f"Motor {names[idx]}: Step {steps[idx]} was outside [0, 4095] before final send. Clipped to {steps_final_clipped[idx]}.", 
                      throttle_duration_sec=5.0
                    )

        self.group_writer.clearParam()
        success_add = True
        for i, name in enumerate(names):
            if name not in LEADER_ARM_MOTORS: continue # Should not happen if MOTORS_TO_COMMAND is used
            motor_id = LEADER_ARM_MOTORS[name][0]
            step_value = int(steps_final_clipped[i])
            param = [scs.SCS_LOBYTE(step_value), scs.SCS_HIBYTE(step_value)]
            if not self.group_writer.addParam(motor_id, param):
                self.get_logger().error(f"GroupSyncWrite addParam failed for {name}(ID {motor_id})")
                success_add = False
        
        if not success_add: return False
        
        comm_result = self.group_writer.txPacket()
        if comm_result != scs.COMM_SUCCESS:
            error_msg = f"GroupSyncWrite txPacket error code: {comm_result}"
            # Attempt to get more descriptive error if packet_handler supports it
            if hasattr(self.packet_handler, 'getTxRxResult'):
                 try: error_msg = f"GroupSyncWrite txPacket error: {self.packet_handler.getTxRxResult(comm_result)}"
                 except Exception: pass # Keep generic error if getTxRxResult fails
            self.get_logger().error(error_msg, throttle_duration_sec=1.0)
            return False
        return True

    def control_signal_callback(self, msg: Vector3):
        # start_time_loop = self.get_clock().now() # For detailed loop timing if needed

        # Ensure node and SDK are ready
        if not (self.port_handler and self.packet_handler and self.calibration_data and \
                self.group_writer and self.group_reader and self.reader_motors_ok):
            self.get_logger().warn("Node not fully initialized or reader setup failed, skipping command.", throttle_duration_sec=5)
            return
        if scs is None: # Should have been caught in __init__
            self.get_logger().error("scservo_sdk not loaded, cannot execute callback.", throttle_duration_sec=5)
            return

        signal_pan = msg.x  # Control signal for Pan movement
        signal_lift_y_cartesian = msg.y # Control signal for Y Cartesian movement (input to Y PI controller)

        # --- 1. Read Current Motor State (Throttled) ---
        raw_positions_np = None
        read_attempted_this_cycle = False
        current_callback_time = self.get_clock().now()
        if self.read_period <= 0 or (current_callback_time - self.last_read_time).nanoseconds / 1e9 >= self.read_period:
             read_attempted_this_cycle = True
             self.last_read_time = current_callback_time
             try:
                 comm_result = self.group_reader.txRxPacket()
                 if comm_result != scs.COMM_SUCCESS:
                     # Log read error, but try to use last known position if available
                     self.get_logger().warn(f"GroupSyncRead txRxPacket error: {comm_result}", throttle_duration_sec=1.0)
                 else:
                     addr_present_pos, len_present_pos = SCS_CONTROL_TABLE["Present_Position"]
                     temp_raw_list = []
                     all_data_available_from_read = True
                     for motor_name_to_read in MOTORS_TO_COMMAND:
                          motor_id = LEADER_ARM_MOTORS[motor_name_to_read][0]
                          if self.group_reader.isAvailable(motor_id, addr_present_pos, len_present_pos):
                               value = self.group_reader.getData(motor_id, addr_present_pos, len_present_pos)
                               temp_raw_list.append(np.int32(value))
                          else:
                               self.get_logger().warn(f"Data not available for {motor_name_to_read}({motor_id}) after GroupSyncRead.", throttle_duration_sec=1.0)
                               temp_raw_list.append(np.nan) # Mark as NaN if not available
                               all_data_available_from_read = False
                     if all_data_available_from_read:
                         raw_positions_np = np.array(temp_raw_list, dtype=np.int32)
                         self.last_known_raw_positions = raw_positions_np # Update last known good read
             except Exception as e_read_stage:
                 self.get_logger().error(f"Exception during GroupSyncRead stage: {e_read_stage}")
        
        # Use last known positions if current read failed or was skipped
        if raw_positions_np is None:
            if self.last_known_raw_positions is not None:
                raw_positions_np = self.last_known_raw_positions
                if read_attempted_this_cycle: self.get_logger().warn("Using last known motor positions due to current read failure/issue.")
            else:
                self.get_logger().error("No current or previous motor state available. Cannot proceed with control.")
                return # Critical: cannot control without knowing current state

        # --- 2. Apply Calibration to Get Current Angles ---
        try:
            calibrated_angles_deg = apply_calibration(
                raw_positions_np, MOTORS_TO_COMMAND, self.calibration_data, LEADER_ARM_MOTORS
            )
            if calibrated_angles_deg is None or np.isnan(calibrated_angles_deg).any():
                self.get_logger().error(f"Apply_calibration failed. Raw:{raw_positions_np} -> Calib:{calibrated_angles_deg}. Skipping control.", throttle_duration_sec=1.0)
                # Attempt to hold last sent steps if calibration fails
                if self.last_sent_target_steps is not None:
                     if not self.send_goal_positions(MOTORS_TO_COMMAND, self.last_sent_target_steps):
                          self.get_logger().error("Failed to send hold command after calibration failure.")
                return
            current_pan_deg = calibrated_angles_deg[0]
            current_theta1_deg = calibrated_angles_deg[1] # Current Shoulder/Lift angle (ID2)
            current_theta2_deg = calibrated_angles_deg[2] # Current Elbow angle (ID3)
        except Exception as e_apply_calib_stage:
            self.get_logger().error(f"Exception during apply_calibration stage: {e_apply_calib_stage}")
            return

        # --- 3. Calculate Target Angles (with Active Hold Logic and Control) ---
        # Initialize target angles with last valid ones (for hold) or current ones if no prior
        if self.last_valid_target_angles is not None:
             target_pan_deg = self.last_valid_target_angles[0]
             target_theta1_deg = self.last_valid_target_angles[1] # Will be updated by IK + Compensation
             target_theta2_deg = self.last_valid_target_angles[2] # Will be updated by IK
        else: # Should only happen on first run if initial_hold_pose failed
             target_pan_deg = current_pan_deg
             target_theta1_deg = current_theta1_deg
             target_theta2_deg = current_theta2_deg
        
        new_target_calculated_this_cycle = False

        # Pan Control (Proportional)
        if abs(signal_pan) >= self.stop_threshold:
            delta_pan_deg = self.pan_increment_scale * signal_pan
            target_pan_deg = current_pan_deg + delta_pan_deg # Target is relative to current
            target_pan_deg = np.clip(target_pan_deg, self.pan_min, self.pan_max)
            new_target_calculated_this_cycle = True

        # Vertical Control (Cartesian Y PI -> IK -> ID2 Angular Compensation PI)
        if abs(signal_lift_y_cartesian) >= self.stop_threshold:
            try:
                current_xw_fk, current_yw_fk = calculate_fk_wrist(current_theta1_deg, current_theta2_deg)

                # PI Control for Y Cartesian Target
                time_y_pi = self.get_clock().now()
                dt_y_pi_duration = time_y_pi - self.last_y_cartesian_control_time
                dt_y_pi = dt_y_pi_duration.nanoseconds / 1e9 
                self.last_y_cartesian_control_time = time_y_pi

                # signal_lift_y_cartesian: positive implies "target is visually higher, need to move Y up"
                self.integral_accumulator_y_cartesian += signal_lift_y_cartesian * dt_y_pi
                
                p_contrib_y_cartesian = self.Kp_y_cartesian * signal_lift_y_cartesian
                i_contrib_y_cartesian_unclipped = self.Ki_y_cartesian * self.integral_accumulator_y_cartesian
                i_contrib_y_cartesian_clipped = np.clip(
                    i_contrib_y_cartesian_unclipped, 
                    self.integral_y_cartesian_min_output_val, 
                    self.integral_y_cartesian_max_output_val
                )
                delta_yw_cartesian_target = p_contrib_y_cartesian + i_contrib_y_cartesian_clipped
                
                target_yw_for_ik = current_yw_fk + delta_yw_cartesian_target
                target_xw_for_ik = current_xw_fk # Assume no X change from vertical control for now

                # Clip Cartesian Y target for IK
                target_yw_for_ik_clipped = np.clip(target_yw_for_ik, self.Y_MIN_LIMIT, self.Y_MAX_LIMIT)
                if abs(target_yw_for_ik_clipped - target_yw_for_ik) > 1e-4:
                    self.get_logger().warn(f"CARTESIAN Y TARGET CLIPPED: Raw={target_yw_for_ik:.4f} -> Clipped={target_yw_for_ik_clipped:.4f}")

                # Inverse Kinematics
                ik_theta1_rad, ik_theta2_rad, ik_ok = calculate_ik(target_xw_for_ik, target_yw_for_ik_clipped)

                if ik_ok:
                    target_th1_deg_from_ik = math.degrees(ik_theta1_rad)
                    target_th2_deg_from_ik = math.degrees(ik_theta2_rad)

                    # --- ID2 (Shoulder/Lift) Angular PI Compensation ---
                    error_angle_id2 = target_th1_deg_from_ik - current_theta1_deg
                    
                    time_id2_comp = self.get_clock().now()
                    dt_id2_comp_duration = time_id2_comp - self.last_comp_angle_id2_time
                    dt_id2_comp = dt_id2_comp_duration.nanoseconds / 1e9
                    self.last_comp_angle_id2_time = time_id2_comp

                    self.integral_accumulator_angle_id2 += error_angle_id2 * dt_id2_comp
                    
                    p_adj_id2 = self.Kp_angular_comp_ID2 * error_angle_id2
                    i_adj_id2_unclipped = self.Ki_angular_comp_ID2 * self.integral_accumulator_angle_id2
                    i_adj_id2_clipped = np.clip(
                        i_adj_id2_unclipped, 
                        self.integral_angular_comp_ID2_min_output_adj_val,
                        self.integral_angular_comp_ID2_max_output_adj_val
                    )
                    total_angle_adjustment_id2 = p_adj_id2 + i_adj_id2_clipped
                    target_theta1_deg_compensated = target_th1_deg_from_ik + total_angle_adjustment_id2
                    
                    self.get_logger().info( # Keep this important log for ID2 compensation behavior
                        f"ID2 Ang.Comp: Err_th1={error_angle_id2:.1f}d, "
                        f"TotalAdj={total_angle_adjustment_id2:.1f}d, "
                        f"Th1_IK={target_th1_deg_from_ik:.1f}d -> Th1_Final={target_theta1_deg_compensated:.1f}d"
                    )
                    # --- End ID2 Angular PI Compensation ---

                    # Apply software joint limits to final target angles
                    target_theta1_deg_final_clipped = np.clip(target_theta1_deg_compensated, self.lift_min, self.lift_max)
                    target_theta2_deg_final_clipped = np.clip(target_th2_deg_from_ik, self.elbow_min, self.elbow_max) # ID3 uses IK output directly
                    
                    if abs(target_theta1_deg_final_clipped - target_theta1_deg_compensated) > 0.1:
                         self.get_logger().warn(f"LIFT (ID2) ANGLE CLIPPED (Post-Comp): Comp={target_theta1_deg_compensated:.1f} -> SW Lim Clip: {target_theta1_deg_final_clipped:.1f}")
                    if abs(target_theta2_deg_final_clipped - target_th2_deg_from_ik) > 0.1: 
                         self.get_logger().warn(f"ELBOW (ID3) ANGLE CLIPPED: IK Raw={target_th2_deg_from_ik:.1f} -> SW Lim Clip: {target_theta2_deg_final_clipped:.1f}")
                    
                    target_theta1_deg = target_theta1_deg_final_clipped
                    target_theta2_deg = target_theta2_deg_final_clipped
                    new_target_calculated_this_cycle = True
                else: # IK failed
                    self.get_logger().warn(f"IK FAILED for target X={target_xw_for_ik:.3f}, Y={target_yw_for_ik_clipped:.3f}. Holding angles.", throttle_duration_sec=1.0)
                    # Reset integrators if IK fails to prevent wind-up
                    self.integral_accumulator_y_cartesian = 0.0
                    self.integral_accumulator_angle_id2 = 0.0
            except Exception as e_vertical_control:
                 self.get_logger().error(f"Exception during vertical control (FK/IK/Compensation) stage: {e_vertical_control}")
                 traceback.print_exc()

        # --- 4. Convert Final Target Angles to Steps and Send ---
        try:
            final_target_angles_deg_np = np.array([target_pan_deg, target_theta1_deg, target_theta2_deg], dtype=np.float32)
            target_steps = revert_calibration(
                final_target_angles_deg_np, MOTORS_TO_COMMAND, self.calibration_data, LEADER_ARM_MOTORS
            )
            if target_steps is None: # Should not happen if revert_calibration returns a default
                self.get_logger().error("Revert calibration returned None. Attempting to hold last sent steps.")
                if self.last_sent_target_steps is not None:
                    target_steps = self.last_sent_target_steps
                else: # Critical failure if no steps to send
                     self.get_logger().error("Cannot hold: No previous steps stored. Skipping write command.")
                     return
            
            log_prefix = "COMMAND (NEW TGT):" if new_target_calculated_this_cycle else "COMMAND (HOLDING):"
            self.get_logger().info(
                f"{log_prefix} Final Angles(deg): {np.round(final_target_angles_deg_np, 1)} -> Steps: {target_steps}"
            )
            
            send_ok = self.send_goal_positions(MOTORS_TO_COMMAND, target_steps)
            if send_ok: # Only update last_valid if send was successful
                if new_target_calculated_this_cycle:
                    self.last_valid_target_angles = final_target_angles_deg_np.copy()
                # Always update last_sent_target_steps if send was OK
                self.last_sent_target_steps = target_steps.copy()
            else: # Send failed
                 self.get_logger().warn("Failed to send target positions. Last valid target NOT updated.")
        except Exception as e_send_stage:
            self.get_logger().error(f"Exception during revert_calibration or write stage: {e_send_stage}")
            traceback.print_exc()

        # loop_duration_ms = (self.get_clock().now() - start_time_loop).nanoseconds / 1e6
        # self.get_logger().debug(f"Control loop duration: {loop_duration_ms:.2f} ms")

    def destroy_node(self):
        self.get_logger().info(f"Shutting down {NODE_NAME} node...")
        # Disable torque on all motors before closing port
        if self.port_handler is not None and self.packet_handler is not None and scs is not None:
             addr_torque = SCS_CONTROL_TABLE["Torque_Enable"][0]
             self.get_logger().info(f"Attempting to disable torque for all arm motors...")
             for name, motor_info in LEADER_ARM_MOTORS.items(): # Iterate through all defined motors
                  motor_id = motor_info[0]
                  try:
                      # Try twice to disable torque, as sometimes the first attempt might fail
                      for attempt in range(2): 
                           scs_comm_result, scs_error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, addr_torque, 0)
                           if scs_comm_result == scs.COMM_SUCCESS and scs_error == 0: break
                           time.sleep(0.01) # Small delay before retry
                      if not(scs_comm_result == scs.COMM_SUCCESS and scs_error == 0):
                           self.get_logger().warn(f"Could not disable torque for {name}({motor_id}) after retries. Res:{scs_comm_result}, Err:{scs_error}")
                  except Exception as e_torque_disable:
                       self.get_logger().warn(f"Exception disabling torque for {name}({motor_id}): {e_torque_disable}")
                  time.sleep(0.02) # Stagger torque disable commands
        
        self.cleanup_resources() # Close port and other resources
        super().destroy_node()
        self.get_logger().info(f"{NODE_NAME} node shut down complete.")

def main(args=None):
    rclpy.init(args=args)
    node = None
    logger = rclpy.logging.get_logger(f'{NODE_NAME}_main') # Use a specific logger for main
    try:
        node = RealRobotInterfaceNode()
        # A more robust check to ensure node initialization was fully successful
        if not (node.port_handler and node.packet_handler and node.calibration_data and \
                node.group_writer and node.group_reader and node.control_signal_subscription and \
                node.reader_motors_ok): # Add other critical components if any
             logger.fatal("Node object created, but critical components are missing/uninitialized.")
             raise RuntimeError("Node initialization incomplete.")
        
        logger.info("RealRobotInterfaceNode initialization successful, starting spin loop.")
        rclpy.spin(node)
    except KeyboardInterrupt:
         logger.info('Ctrl+C detected, initiating shutdown.')
    except RuntimeError as e_runtime: # Catch specific RuntimeError from init
         logger.fatal(f"Caught RuntimeError during node setup: {e_runtime}")
         # traceback.print_exc() # Already printed in __init__ if it originated there
    except Exception as e_unhandled: # Catch any other unexpected exceptions
        logger.fatal(f'Unhandled exception in main execution: {e_unhandled}')
        traceback.print_exc()
    finally:
        if node is not None:
            logger.info("Executing final node cleanup from main...")
            node.destroy_node() # This will call the class's destroy_node
        else:
            logger.warn("Node object was None or not fully created, skipping destroy_node call from main.")
        
        if rclpy.ok(): # Check if RCLPY context is still valid
            rclpy.shutdown()
            logger.info("RCLPY shutdown complete.")
        print(f"{NODE_NAME} main process finished.")

if __name__ == '__main__':
    main()