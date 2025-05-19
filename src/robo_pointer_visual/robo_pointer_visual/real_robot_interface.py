# Fichier: ~/ros2_ws/src/robo_pointer_visual/robo_pointer_visual/real_robot_interface.py
# Version: Finalized_V1_With_Full_JointState_Offsets

import numpy as np
import math
import time
import json
import os
import xacro
from pathlib import Path
import enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
import traceback 
from ament_index_python.packages import get_package_share_directory
from urdf_parser_py.urdf import URDF

# Utilisation directe de FeetechMotorsBus
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus, FeetechMotorsBusConfig
# Importation directe du SDK bas niveau
try:
    import scservo_sdk as scs
    from scservo_sdk import GroupSyncRead, GroupSyncWrite
except ImportError:
    # This is a critical error, log and allow __init__ to raise it
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

LEADER_ARM_PORT = "/dev/ttyACM0" # Default port, consider making it a ROS parameter
LEADER_ARM_MOTORS = {
    "shoulder_pan": [1, "sts3215"],
    "shoulder_lift": [2, "sts3215"],
    "elbow_flex": [3, "sts3215"],
    "wrist_flex": [4, "sts3215"],
    "wrist_roll": [5, "sts3215"],
    "gripper": [6, "sts3215"],
}
# Order for calibration file and internal consistency
MOTOR_NAMES_ORDER = list(LEADER_ARM_MOTORS.keys())
MODEL_RESOLUTION = {"sts3215": 4096} # Steps per full revolution for sts3215
HALF_TURN_DEGREE = 180.0

# Feetech Servo Control Table Addresses and Lengths
SCS_CONTROL_TABLE = {
    "Torque_Enable": (40, 1),
    "Goal_Position": (42, 2),
    "Present_Position": (56, 2),
    "Status_Return_Level": (16, 1), # To disable reply packets
}

# Motors actively controlled by the IK/PI logic
MOTORS_TO_COMMAND = ["shoulder_pan", "shoulder_lift", "elbow_flex"]
# Initial pose for the controlled motors when holding
INITIAL_HOLD_POSE_DEG = np.array([0.0, 90.0, 90.0], dtype=np.float32)
# Initial position for wrist_flex if set independently at startup
INITIAL_WRIST_POS_DEG = 0.0

class CalibrationMode(enum.Enum):
    DEGREE = 0
    LINEAR = 1

# --- Fonctions de Calibration (Identiques à la version précédente) ---
def apply_calibration(raw_values: np.ndarray, motor_names: list[str], calibration_data: dict, motors_def: dict):
    if calibration_data is None:
        # Logged by caller if necessary
        return np.full(len(motor_names), np.nan, dtype=np.float32)

    calibrated_values = np.full(len(motor_names), np.nan, dtype=np.float32)

    for i, name in enumerate(motor_names):
        if name not in calibration_data["motor_names"]:
            continue

        try:
            calib_idx = calibration_data["motor_names"].index(name)
            if not (calib_idx < len(calibration_data.get("drive_mode", [])) and \
                    calib_idx < len(calibration_data.get("homing_offset", [])) and \
                    calib_idx < len(calibration_data.get("calib_mode", []))):
                # This indicates a malformed calibration file, error should be logged by caller
                continue

            calib_mode_str = calibration_data["calib_mode"][calib_idx]
            try: calib_mode = CalibrationMode[calib_mode_str.upper()] # Ensure uppercase for enum matching
            except KeyError:
                continue # Invalid calib_mode string

            drive_mode = calibration_data["drive_mode"][calib_idx]
            homing_offset = float(calibration_data["homing_offset"][calib_idx])
            _, model = motors_def[name] # Assumes motors_def has all names in motor_names
            resolution = float(MODEL_RESOLUTION[model])
            current_raw_value = float(raw_values[i])
            steps_per_180_deg = resolution / 2.0

            if calib_mode == CalibrationMode.DEGREE:
                if abs(steps_per_180_deg) < 1e-9: continue # Avoid division by zero

                raw_zero = -homing_offset
                delta_raw = current_raw_value - raw_zero
                
                delta_normalized = delta_raw
                if delta_raw > steps_per_180_deg: delta_normalized = delta_raw - resolution
                elif delta_raw < -steps_per_180_deg: delta_normalized = delta_raw + resolution
                
                delta_steps_calibrated = delta_normalized
                if drive_mode == 1: delta_steps_calibrated *= -1.0

                calibrated_degrees = delta_steps_calibrated / steps_per_180_deg * HALF_TURN_DEGREE
                calibrated_values[i] = calibrated_degrees

            elif calib_mode == CalibrationMode.LINEAR:
                 if not (calib_idx < len(calibration_data.get("start_pos", [])) and \
                         calib_idx < len(calibration_data.get("end_pos", []))):
                      continue
                 start_pos = float(calibration_data["start_pos"][calib_idx])
                 end_pos = float(calibration_data["end_pos"][calib_idx])
                 if abs(end_pos - start_pos) < 1e-6:
                      calibrated_values[i] = 0.0 if current_raw_value == start_pos else 50.0
                 else:
                      percentage = (current_raw_value - start_pos) / (end_pos - start_pos) * 100.0
                      calibrated_values[i] = np.clip(percentage, -10.0, 110.0)
        except Exception:
            # Error during processing a specific motor, log handled by caller if needed
            # traceback.print_exc() # Uncomment for deep debugging if errors persist here
            pass # Continue to next motor
    return calibrated_values

def revert_calibration(target_values_calibrated: np.ndarray, motor_names: list[str], calibration_data: dict, motors_def: dict):
    if calibration_data is None:
         return None # Logged by caller

    target_steps = np.zeros(len(motor_names), dtype=np.int32)

    for i, name in enumerate(motor_names):
        motor_model_tuple = motors_def.get(name, ["", "sts3215"])
        default_step = MODEL_RESOLUTION.get(motor_model_tuple[1], 4096) // 2

        if name not in calibration_data["motor_names"]:
            target_steps[i] = default_step
            continue

        try:
            calib_idx = calibration_data["motor_names"].index(name)
            if not (calib_idx < len(calibration_data.get("drive_mode", [])) and \
                    calib_idx < len(calibration_data.get("homing_offset", [])) and \
                    calib_idx < len(calibration_data.get("calib_mode", []))):
                target_steps[i] = default_step
                continue

            calib_mode_str = calibration_data["calib_mode"][calib_idx]
            try: calib_mode = CalibrationMode[calib_mode_str.upper()]
            except KeyError:
                target_steps[i] = default_step
                continue

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
            target_step_raw = float(default_step)

            if calib_mode == CalibrationMode.DEGREE:
                steps_per_180_deg = resolution / 2.0
                if abs(steps_per_180_deg) < 1e-9: continue
                step_centered = current_target_calibrated / HALF_TURN_DEGREE * steps_per_180_deg
                if drive_mode == 1: step_centered *= -1.0
                target_step_raw = step_centered - homing_offset
            elif calib_mode == CalibrationMode.LINEAR:
                 if not apply_hw_clip_from_json:
                      target_steps[i] = default_step
                      continue
                 start_pos_json = hw_min_step_json
                 end_pos_json = hw_max_step_json
                 if abs(end_pos_json - start_pos_json) < 1e-6:
                     target_step_raw = start_pos_json
                 else:
                     target_percentage_clipped = np.clip(current_target_calibrated, 0.0, 100.0)
                     target_step_raw = (target_percentage_clipped / 100.0) * (end_pos_json - start_pos_json) + start_pos_json

            final_step_value_clipped_json = target_step_raw
            if apply_hw_clip_from_json:
                final_step_value_clipped_json = np.clip(target_step_raw, hw_min_step_json, hw_max_step_json)
            
            final_step_value_abs_clipped = int(round(np.clip(final_step_value_clipped_json, 0, resolution - 1)))
            target_steps[i] = final_step_value_abs_clipped

        except Exception:
            # traceback.print_exc() # Uncomment for deep debugging
            target_steps[i] = default_step
    return target_steps


class RealRobotInterfaceNode(Node):
    """
    ROS 2 Node to interface with the real robot arm (SO-100 or similar Feetech-based).
    It subscribes to control signals, reads motor states, calculates target motor commands
    using Inverse Kinematics (IK) and Proportional-Integral (PI) control,
    and sends these commands to the motors. It also publishes the robot's
    joint states for visualization in RViz.
    """
    def __init__(self):
        super().__init__(NODE_NAME)
        self.get_logger().info(f"{NODE_NAME} node starting... (Version: Finalized_V1_With_Full_JointState_Offsets)")
        
        # Initialize all instance attributes to ensure they exist
        self.leader_bus_instance = None
        self.port_handler = None
        self.packet_handler = None
        self.calibration_data = None
        self.group_writer = None
        self.group_reader = None
        self.control_signal_subscription = None
        self.joint_state_publisher = None
        self.reader_motors_ok = False
        self.last_valid_target_angles = None
        self.last_sent_target_steps = None
        self.last_known_raw_positions = None
        self.last_read_time = self.get_clock().now()

        # URDF joint names in the order they appear in the URDF file for JointState messages
        self.joint_names_urdf_order = ['Rotation', 'Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll', 'Jaw']
        
        # Mapping from internal motor names to URDF joint names
        self.active_motor_names_map = {
            "shoulder_pan": "Rotation",
            "shoulder_lift": "Pitch",
            "elbow_flex": "Elbow",
            "wrist_flex": "Wrist_Pitch",
            "wrist_roll": "Wrist_Roll",
            "gripper": "Jaw"
        }

        # Offsets to align physical robot zero with URDF zero (in radians)
        self.joint_offsets_rad = {
            'Rotation': 0.0000,    # shoulder_pan (ID1)
            'Pitch': -1.6889,      # shoulder_lift (ID2)
            'Elbow': -1.5186,      # elbow_flex (ID3)
            'Wrist_Pitch': -1.5156,# wrist_flex (ID4)
            'Wrist_Roll': 0.0,     # wrist_roll (ID5) - Calibrate if needed
            'Jaw': 0.0             # gripper (ID6) - Calibrate if needed
        }

        self.robot_model_urdf = None
        self.link_inertial_params = {}

        if scs is None or GroupSyncRead is None or GroupSyncWrite is None:
             self.get_logger().fatal("scservo_sdk or required Group classes not found. Aborting initialization.")
             raise ImportError("scservo_sdk is required but not installed or incomplete.")

        try:
            self._load_calibration_file()
            self._connect_motor_bus()
            self._configure_motors_srl()
            self._initialize_group_sync_handlers()
            self._load_and_parse_urdf_parameters()
            self._initialize_motors_torque_and_pose()
            self._declare_and_get_ros_parameters()
            
            self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
            self.control_signal_subscription = self.create_subscription(
                Vector3, '/robot_control_signal', self.control_signal_callback, 10
            )
            self.get_logger().info('Successfully subscribed to /robot_control_signal and created /joint_states publisher.')

        except Exception as e_init:
            self.get_logger().fatal(f"CRITICAL ERROR during node initialization: {e_init}")
            traceback.print_exc()
            self.cleanup_resources()
            raise RuntimeError("Node initialization failed") from e_init

    def _load_calibration_file(self):
        try:
            package_share_directory = get_package_share_directory(PACKAGE_NAME)
            calibration_file_path = Path(package_share_directory) / 'config' / DEFAULT_CALIBRATION_FILENAME
        except Exception as e:
            self.get_logger().fatal(f"Could not find package share directory '{PACKAGE_NAME}' or construct path: {e}")
            raise RuntimeError("Failed to get calibration file path") from e
        
        if not calibration_file_path.exists():
            self.get_logger().error(f"Calibration file not found: {calibration_file_path}.")
            raise FileNotFoundError(f"Calibration file not found: {calibration_file_path}")
        with open(calibration_file_path, 'r') as f:
            self.calibration_data = json.load(f)
        self.get_logger().info(f"Calibration data loaded from: {calibration_file_path}")
        
        expected_keys = ["motor_names", "homing_offset", "drive_mode", "calib_mode", "start_pos", "end_pos"]
        if not all(k in self.calibration_data for k in expected_keys):
             self.get_logger().error("Calibration file missing required keys. Check structure.")
             raise ValueError("Invalid calibration file structure.")
        if self.calibration_data.get("motor_names") != MOTOR_NAMES_ORDER:
             self.get_logger().error("Motor order mismatch in calibration file vs. MOTOR_NAMES_ORDER.")
             raise ValueError("Motor order mismatch in calibration file.")

    def _connect_motor_bus(self):
        leader_config = FeetechMotorsBusConfig(port=LEADER_ARM_PORT, motors=LEADER_ARM_MOTORS)
        self.leader_bus_instance = FeetechMotorsBus(leader_config)
        self.get_logger().info(f"Connecting to bus on {LEADER_ARM_PORT}...")
        self.leader_bus_instance.connect()
        self.port_handler = self.leader_bus_instance.port_handler
        self.packet_handler = self.leader_bus_instance.packet_handler
        if self.port_handler is None or self.packet_handler is None:
             raise RuntimeError("Failed to get valid port/packet handlers from FeetechMotorsBus.")
        self.get_logger().info("Successfully connected to motor bus.")

    def _configure_motors_srl(self):
        addr_ret_level = SCS_CONTROL_TABLE["Status_Return_Level"][0]
        self.get_logger().info(f"Setting Status Return Level to 0 for motors: {MOTOR_NAMES_ORDER}...")
        # Setting SRL for all motors defined in LEADER_ARM_MOTORS for robustness
        for name in MOTOR_NAMES_ORDER: 
             if name not in LEADER_ARM_MOTORS: continue
             motor_id = LEADER_ARM_MOTORS[name][0]
             scs_comm_result, scs_error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, addr_ret_level, 0)
             if scs_comm_result != scs.COMM_SUCCESS or scs_error != 0:
                  self.get_logger().warn(f"Failed to set Status Return Level for {name}(ID:{motor_id}). Res:{scs_comm_result}, Err:{scs_error}")
             time.sleep(0.01) # Small delay

    def _initialize_group_sync_handlers(self):
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
        # Add all motors we might want to read (currently MOTORS_TO_COMMAND)
        for name in MOTORS_TO_COMMAND: 
            if name not in LEADER_ARM_MOTORS: continue
            motor_id = LEADER_ARM_MOTORS[name][0]
            if not self.group_reader.addParam(motor_id):
                self.get_logger().error(f"Initial GroupSyncRead addParam failed for {name}(ID:{motor_id})")
                self.reader_motors_ok = False
        if not self.reader_motors_ok:
             self.get_logger().error("Could not add all MOTORS_TO_COMMAND to GroupSyncRead handler.")
             # This might not be fatal if control logic can handle missing reads, but it's a problem.

    def _load_and_parse_urdf_parameters(self):
        """
        Loads the robot's URDF file from the 'so100_description' package,
        parses it, and extracts inertial parameters for relevant links.
        """
        self.get_logger().info("Loading and parsing URDF for inertial parameters...")

        links_to_extract_inertial_data = [
            "Upper_Arm",        # Enfant du joint 'Pitch' (ID2)
            "Lower_Arm",        # Enfant du joint 'Elbow' (ID3)
            "Wrist_Pitch_Roll", # Enfant du joint 'Wrist_Pitch' (ID4)
            "Fixed_Jaw",        # Enfant du joint 'Wrist_Roll' (ID5)
            "Moving_Jaw"        # Enfant du joint 'Jaw' (ID6)
        ]

        try:
            so100_description_pkg_share_dir = get_package_share_directory("so100_description")
            urdf_file_name_in_pkg = 'SO_5DOF_ARM100_8j_URDF.SLDASM.urdf'
            full_urdf_path = os.path.join(so100_description_pkg_share_dir, 'urdf', urdf_file_name_in_pkg)
            self.get_logger().info(f"Attempting to process URDF/XACRO from: {full_urdf_path}")
            
            xml_doc = xacro.process_file(full_urdf_path)
            robot_description_xml_str = xml_doc.toprettyxml(indent="    ")
            self.robot_model_urdf = URDF.from_xml_string(robot_description_xml_str)
            self.get_logger().info("URDF model loaded and parsed successfully.")
            
            for link_name in links_to_extract_inertial_data:
                link_object = self.robot_model_urdf.get_link(link_name)
                
                if link_object and link_object.inertial:
                    self.link_inertial_params[link_name] = {
                        "mass": link_object.inertial.mass,
                        "com_in_link_frame": list(link_object.inertial.origin.xyz)
                    }
                    self.get_logger().info(
                        f"  Extracted for Link '{link_name}': "
                        f"Mass: {self.link_inertial_params[link_name]['mass']}, "
                        f"COM in Link Frame: {self.link_inertial_params[link_name]['com_in_link_frame']}")
                else:
                    self.get_logger().warn(f"Link '{link_name}' not found or missing inertial data.")
                    self.link_inertial_params[link_name] = {'mass': 0.0, 'com_in_link_frame': [0.0, 0.0, 0.0]}
            
        except FileNotFoundError:
            self.get_logger().error(f"URDF file not found at processed path derived from '{full_urdf_path}'. Cannot load inertial params.")
            self._set_default_inertial_params(links_to_extract_inertial_data)
        
        except Exception as e:
            self.get_logger().error(f"Failed to load or parse URDF: {e}")
            traceback.print_exc()
            self._set_default_inertial_params(links_to_extract_inertial_data)
    
    def _set_default_inertial_params(self, link_names_list):
        """Helper to set default inertial params if URDF parsing fails."""
        self.get_logger().warn("Setting default (zero mass) inertial parameters due to URDF loading/parsing failure.")
        for link_name in link_names_list:
            self.link_inertial_params[link_name] = {'mass': 0.0, 'com_in_link_frame': [0.0, 0.0, 0.0]}

    def _initialize_motors_torque_and_pose(self):
        motors_to_init_torque = MOTOR_NAMES_ORDER # Enable torque for all motors
        self.get_logger().info(f"Enabling torque for: {motors_to_init_torque}...")
        addr_torque = SCS_CONTROL_TABLE["Torque_Enable"][0]
        addr_goal_pos_init = SCS_CONTROL_TABLE["Goal_Position"][0]
        
        for name in motors_to_init_torque:
            if name not in LEADER_ARM_MOTORS: continue
            motor_id = LEADER_ARM_MOTORS[name][0]
            scs_comm_result, scs_error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, addr_torque, 1)
            if scs_comm_result != scs.COMM_SUCCESS or (scs_error != 0 and scs_error != 32):
                 self.get_logger().warn(f"Torque enable for {name}(ID:{motor_id}) potentially failed. Res:{scs_comm_result}, Err:{scs_error}")
            time.sleep(0.02)
            
            # Set initial position for wrist_flex separately if needed (and if it's not in MOTORS_TO_COMMAND for main hold)
            if name == "wrist_flex" and name not in MOTORS_TO_COMMAND:
                target_steps_wrist_array = revert_calibration(np.array([INITIAL_WRIST_POS_DEG]), [name], self.calibration_data, LEADER_ARM_MOTORS)
                if target_steps_wrist_array is not None:
                    target_steps_wrist = target_steps_wrist_array.item()
                    self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, addr_goal_pos_init, target_steps_wrist)
                    time.sleep(0.1) # Allow wrist to reach initial pos
        
        self.get_logger().info(f"Calculating initial hold target pose for controlled motors: {INITIAL_HOLD_POSE_DEG} deg")
        initial_hold_steps = revert_calibration(
            INITIAL_HOLD_POSE_DEG, MOTORS_TO_COMMAND, self.calibration_data, LEADER_ARM_MOTORS
        )
        if initial_hold_steps is not None:
            self.last_valid_target_angles = INITIAL_HOLD_POSE_DEG.copy()
            self.last_sent_target_steps = initial_hold_steps
            self.get_logger().info(f"Initial hold target for controlled motors set -> Angles: {np.round(self.last_valid_target_angles,1)}, Steps: {self.last_sent_target_steps}")
        else:
            self.get_logger().error("Failed to calculate initial hold steps. Hold logic might start passively.")
        self.get_logger().info("Motor torque and initial pose setup complete.")

    def _declare_and_get_ros_parameters(self):
        # General Control Parameters
        self.declare_parameter('stop_threshold', 0.01)
        self.declare_parameter('read_frequency_hz', 20.0)
        self.stop_threshold = self.get_parameter('stop_threshold').get_parameter_value().double_value
        read_frequency = self.get_parameter('read_frequency_hz').get_parameter_value().double_value
        self.read_period = 1.0 / read_frequency if read_frequency > 0 else 0.0 # Avoid division by zero

        # Pan Control Parameters
        self.declare_parameter('pan_increment_scale', 0.1)
        self.declare_parameter('pan_angle_min', -110.0)
        self.declare_parameter('pan_angle_max', 110.0)
        self.pan_increment_scale = self.get_parameter('pan_increment_scale').get_parameter_value().double_value
        self.pan_min = self.get_parameter('pan_angle_min').get_parameter_value().double_value
        self.pan_max = self.get_parameter('pan_angle_max').get_parameter_value().double_value

        # Cartesian Y Target Limits for IK input
        self.declare_parameter('target_yw_min', -0.02)
        self.declare_parameter('target_yw_max', 0.52)
        self.Y_MIN_LIMIT = self.get_parameter('target_yw_min').get_parameter_value().double_value
        self.Y_MAX_LIMIT = self.get_parameter('target_yw_max').get_parameter_value().double_value

        # Software Joint Limits for IK output clipping (degrees)
        self.declare_parameter('lift_min', 0.0)
        self.declare_parameter('lift_max', 130.0)
        self.declare_parameter('elbow_min', 0.0)
        self.declare_parameter('elbow_max', 110.0)
        self.lift_min = self.get_parameter('lift_min').get_parameter_value().double_value
        self.lift_max = self.get_parameter('lift_max').get_parameter_value().double_value
        self.elbow_min = self.get_parameter('elbow_min').get_parameter_value().double_value
        self.elbow_max = self.get_parameter('elbow_max').get_parameter_value().double_value
        
        # PI Control for Global Vertical (Y Cartesian) Movement
        self.declare_parameter('kp_vertical_gain', -0.00027)
        self.declare_parameter('ki_vertical_gain', -0.00010)
        self.declare_parameter('integral_max_output', 0.05)
        self.declare_parameter('integral_min_output', -0.05)
        self.Kp_y_cartesian = self.get_parameter('kp_vertical_gain').get_parameter_value().double_value
        self.Ki_y_cartesian = self.get_parameter('ki_vertical_gain').get_parameter_value().double_value
        self.integral_y_cartesian_max_output_val = self.get_parameter('integral_max_output').get_parameter_value().double_value
        self.integral_y_cartesian_min_output_val = self.get_parameter('integral_min_output').get_parameter_value().double_value
        
        # PI Control for ID2 (Shoulder/Lift) Angular Compensation
        self.declare_parameter('kp_angular_comp_id2', 0.0080)
        self.declare_parameter('ki_angular_comp_id2', 0.00025) # Corrected from 0.0003 to 0.00025 as per log
        self.declare_parameter('integral_angular_comp_id2_max_output_adj', 3.0)
        self.declare_parameter('integral_angular_comp_id2_min_output_adj', -3.0)
        self.Kp_angular_comp_ID2 = self.get_parameter('kp_angular_comp_id2').get_parameter_value().double_value
        self.Ki_angular_comp_ID2 = self.get_parameter('ki_angular_comp_id2').get_parameter_value().double_value
        self.integral_angular_comp_ID2_max_output_adj_val = self.get_parameter('integral_angular_comp_id2_max_output_adj').get_parameter_value().double_value
        self.integral_angular_comp_ID2_min_output_adj_val = self.get_parameter('integral_angular_comp_id2_min_output_adj').get_parameter_value().double_value
        
        # Initialize PI controller state variables
        self.integral_accumulator_y_cartesian = 0.0
        self.last_y_cartesian_control_time = self.get_clock().now()
        self.integral_accumulator_angle_id2 = 0.0
        self.last_comp_angle_id2_time = self.get_clock().now()

        self.get_logger().info("ROS Parameters loaded.")
        # Parameters are logged individually if needed by their respective logic blocks.

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
        self.port_handler = None
        self.packet_handler = None
        self.group_writer = None
        self.group_reader = None

    def send_goal_positions(self, motor_names_to_send: list[str], steps_to_send: np.ndarray) -> bool:
        if self.group_writer is None: 
            self.get_logger().error("GroupSyncWrite handler not initialized.", throttle_duration_sec=5)
            return False
        
        steps_final_clipped = np.clip(steps_to_send, 0, MODEL_RESOLUTION["sts3215"] - 1).astype(np.int32)
        
        self.group_writer.clearParam()
        success_add = True
        for i, name in enumerate(motor_names_to_send):
            if name not in LEADER_ARM_MOTORS: continue
            motor_id = LEADER_ARM_MOTORS[name][0]
            step_value = int(steps_final_clipped[i])
            param = [scs.SCS_LOBYTE(step_value), scs.SCS_HIBYTE(step_value)]
            if not self.group_writer.addParam(motor_id, param):
                self.get_logger().error(f"GroupSyncWrite addParam failed for {name}(ID {motor_id})")
                success_add = False
        
        if not success_add: return False
        
        comm_result = self.group_writer.txPacket()
        if comm_result != scs.COMM_SUCCESS:
            error_msg_detail = self.packet_handler.getTxRxResult(comm_result) if hasattr(self.packet_handler, 'getTxRxResult') else f"Code: {comm_result}"
            self.get_logger().error(f"GroupSyncWrite txPacket error: {error_msg_detail}", throttle_duration_sec=1.0)
            return False
        return True

    def _publish_joint_states(self, calibrated_angles_deg_ordered: np.ndarray):
        """Publishes the current joint states for RViz visualization."""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names_urdf_order # ['Rotation', 'Pitch', ..., 'Jaw']
        
        # Initialize positions for all URDF joints (some might not be actively read)
        positions_for_urdf = [0.0] * len(self.joint_names_urdf_order)

        # Map read/calibrated angles (ordered by MOTORS_TO_COMMAND) to URDF joint order
        angles_lus_map_rad = {}
        current_angles_rad_calibrated = np.radians(calibrated_angles_deg_ordered)
        for i, motor_name_internal in enumerate(MOTORS_TO_COMMAND):
            angles_lus_map_rad[motor_name_internal] = current_angles_rad_calibrated[i]

        for i, urdf_joint_name in enumerate(self.joint_names_urdf_order):
            current_motor_angle_rad = 0.0 # Default if not actively read
            
            # Find the internal motor name corresponding to this URDF joint
            internal_motor_name_for_urdf = None
            for motor_internal_map_key, urdf_mapped_val in self.active_motor_names_map.items():
                if urdf_mapped_val == urdf_joint_name:
                    internal_motor_name_for_urdf = motor_internal_map_key
                    break
            
            if internal_motor_name_for_urdf and internal_motor_name_for_urdf in angles_lus_map_rad:
                # This joint's angle was read from MOTORS_TO_COMMAND
                current_motor_angle_rad = angles_lus_map_rad[internal_motor_name_for_urdf]
            
            # Apply the defined offset for this URDF joint
            # Offset defines the URDF zero pose relative to the calibrated motor zero
            angle_to_publish = current_motor_angle_rad + self.joint_offsets_rad.get(urdf_joint_name, 0.0)
            
            # Apply specific inversion for 'Pitch' joint (ID2 / shoulder_lift)
            # This ensures the motion in RViz matches the physical robot's "up" direction
            if urdf_joint_name == self.active_motor_names_map.get("shoulder_lift"): # "Pitch"
                angle_to_publish *= -1.0
            
            positions_for_urdf[i] = float(angle_to_publish)
            
        joint_state_msg.position = positions_for_urdf
        self.joint_state_publisher.publish(joint_state_msg)

    def control_signal_callback(self, msg: Vector3):
        if not (self.port_handler and self.packet_handler and self.calibration_data and \
                self.group_writer and self.group_reader and self.reader_motors_ok):
            self.get_logger().warn("Node components not fully initialized, skipping command.", throttle_duration_sec=5)
            return
        if scs is None:
            self.get_logger().error("scservo_sdk not loaded, cannot execute callback.", throttle_duration_sec=5)
            return

        signal_pan = msg.x
        signal_lift_y_cartesian = msg.y

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
                     self.get_logger().warn(f"GroupSyncRead txRxPacket error. Using last known positions.", throttle_duration_sec=1.0)
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
                               self.get_logger().warn(f"Data not available for {motor_name_to_read}(ID:{motor_id}) after GroupSyncRead. Using last known.", throttle_duration_sec=1.0)
                               temp_raw_list.append(np.nan)
                               all_data_available_from_read = False
                     if all_data_available_from_read:
                         raw_positions_np = np.array(temp_raw_list, dtype=np.int32)
                         self.last_known_raw_positions = raw_positions_np
                     # If not all data available, raw_positions_np remains None for this cycle
             except Exception as e_read_stage:
                 self.get_logger().error(f"Exception during GroupSyncRead: {e_read_stage}")
        
        if raw_positions_np is None:
            if self.last_known_raw_positions is not None:
                raw_positions_np = self.last_known_raw_positions
                if read_attempted_this_cycle: self.get_logger().debug("Using last known motor positions due to current read issue.")
            else:
                self.get_logger().error("No current or previous motor state available. Cannot proceed.")
                return

        # --- 2. Apply Calibration to Get Current Angles for Controlled Motors ---
        try:
            # calibrated_angles_deg will be ordered according to MOTORS_TO_COMMAND
            calibrated_angles_deg = apply_calibration(
                raw_positions_np, MOTORS_TO_COMMAND, self.calibration_data, LEADER_ARM_MOTORS
            )
            if calibrated_angles_deg is None or np.isnan(calibrated_angles_deg).any():
                self.get_logger().error(f"Apply_calibration failed. Raw:{raw_positions_np} -> Calib:{calibrated_angles_deg}. Holding.", throttle_duration_sec=1.0)
                if self.last_sent_target_steps is not None:
                     if not self.send_goal_positions(MOTORS_TO_COMMAND, self.last_sent_target_steps):
                          self.get_logger().error("Failed to send hold command after calibration failure.")
                return
            current_pan_deg = calibrated_angles_deg[0]
            current_theta1_deg = calibrated_angles_deg[1] # Shoulder/Lift (ID2)
            current_theta2_deg = calibrated_angles_deg[2] # Elbow (ID3)
        except Exception as e_apply_calib_stage:
            self.get_logger().error(f"Exception during apply_calibration: {e_apply_calib_stage}")
            return

        # --- Publish Joint States for RViz (using angles of MOTORS_TO_COMMAND) ---
        self._publish_joint_states(calibrated_angles_deg)

        # --- 3. Calculate Target Angles (with Active Hold Logic and Control) ---
        if self.last_valid_target_angles is not None:
             target_pan_deg = self.last_valid_target_angles[0]
             target_theta1_deg = self.last_valid_target_angles[1]
             target_theta2_deg = self.last_valid_target_angles[2]
        else:
             target_pan_deg = current_pan_deg
             target_theta1_deg = current_theta1_deg
             target_theta2_deg = current_theta2_deg
        
        new_target_calculated_this_cycle = False

        # Pan Control
        if abs(signal_pan) >= self.stop_threshold:
            delta_pan_deg = self.pan_increment_scale * signal_pan
            target_pan_deg = np.clip(current_pan_deg + delta_pan_deg, self.pan_min, self.pan_max)
            new_target_calculated_this_cycle = True

        # Vertical Control (Cartesian Y PI -> IK -> ID2 Angular Compensation PI)
        if abs(signal_lift_y_cartesian) >= self.stop_threshold:
            try:
                current_xw_fk, current_yw_fk = calculate_fk_wrist(current_theta1_deg, current_theta2_deg)

                time_y_pi = self.get_clock().now()
                dt_y_pi = (time_y_pi - self.last_y_cartesian_control_time).nanoseconds / 1e9 
                self.last_y_cartesian_control_time = time_y_pi

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
                target_xw_for_ik = current_xw_fk

                target_yw_for_ik_clipped = np.clip(target_yw_for_ik, self.Y_MIN_LIMIT, self.Y_MAX_LIMIT)
                
                ik_theta1_rad, ik_theta2_rad, ik_ok = calculate_ik(target_xw_for_ik, target_yw_for_ik_clipped)

                if ik_ok:
                    target_th1_deg_from_ik = math.degrees(ik_theta1_rad)
                    target_th2_deg_from_ik = math.degrees(ik_theta2_rad)

                    error_angle_id2 = target_th1_deg_from_ik - current_theta1_deg
                    time_id2_comp = self.get_clock().now()
                    dt_id2_comp = (time_id2_comp - self.last_comp_angle_id2_time).nanoseconds / 1e9
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
                    
                    self.get_logger().debug(
                        f"ID2 Ang.Comp: Err_th1={error_angle_id2:.1f}d, TotalAdj={total_angle_adjustment_id2:.1f}d, "
                        f"Th1_IK={target_th1_deg_from_ik:.1f}d -> Th1_FinalComp={target_theta1_deg_compensated:.1f}d"
                    )
                    
                    target_theta1_deg = np.clip(target_theta1_deg_compensated, self.lift_min, self.lift_max)
                    target_theta2_deg = np.clip(target_th2_deg_from_ik, self.elbow_min, self.elbow_max)
                    
                    new_target_calculated_this_cycle = True
                else: 
                    self.get_logger().warn(f"IK FAILED for target X={target_xw_for_ik:.3f}, Y={target_yw_for_ik_clipped:.3f}. Holding.", throttle_duration_sec=1.0)
                    self.integral_accumulator_y_cartesian = 0.0 # Reset integrators
                    self.integral_accumulator_angle_id2 = 0.0
            except Exception as e_vertical_control:
                 self.get_logger().error(f"Exception during vertical control logic: {e_vertical_control}")
                 # traceback.print_exc() # Uncomment for deep debugging

        # --- 4. Convert Final Target Angles to Steps and Send ---
        try:
            final_target_angles_deg_np = np.array([target_pan_deg, target_theta1_deg, target_theta2_deg], dtype=np.float32)
            target_steps = revert_calibration(
                final_target_angles_deg_np, MOTORS_TO_COMMAND, self.calibration_data, LEADER_ARM_MOTORS
            )
            if target_steps is None:
                self.get_logger().error("Revert calibration failed. Attempting to hold last sent steps.")
                if self.last_sent_target_steps is not None:
                    target_steps = self.last_sent_target_steps
                else:
                     self.get_logger().error("Cannot hold: No previous steps stored. Skipping command.")
                     return
            
            log_level = self.get_logger().info if new_target_calculated_this_cycle else self.get_logger().debug
            log_prefix = "COMMAND (NEW TGT):" if new_target_calculated_this_cycle else "COMMAND (HOLDING):"
            log_level(
                f"{log_prefix} Final Angles(deg): {np.round(final_target_angles_deg_np, 1)} -> Steps: {target_steps}"
            )
            
            send_ok = self.send_goal_positions(MOTORS_TO_COMMAND, target_steps)
            if send_ok:
                if new_target_calculated_this_cycle:
                    self.last_valid_target_angles = final_target_angles_deg_np.copy()
                self.last_sent_target_steps = target_steps.copy()
            else:
                 self.get_logger().warn("Failed to send target positions. Last valid target NOT updated.")
        except Exception as e_send_stage:
            self.get_logger().error(f"Exception during revert_calibration or write stage: {e_send_stage}")
            # traceback.print_exc() # Uncomment for deep debugging

    def destroy_node(self):
        self.get_logger().info(f"Shutting down {NODE_NAME} node...")
        if self.port_handler is not None and self.packet_handler is not None and scs is not None:
             addr_torque = SCS_CONTROL_TABLE["Torque_Enable"][0]
             self.get_logger().info(f"Attempting to disable torque for all arm motors...")
             for name, motor_info in LEADER_ARM_MOTORS.items():
                  motor_id = motor_info[0]
                  try:
                      for attempt in range(2): 
                           scs_comm_result, scs_error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, addr_torque, 0)
                           if scs_comm_result == scs.COMM_SUCCESS and scs_error == 0: break
                           time.sleep(0.01)
                      if not(scs_comm_result == scs.COMM_SUCCESS and scs_error == 0):
                           self.get_logger().warn(f"Could not disable torque for {name}(ID:{motor_id}). Res:{scs_comm_result}, Err:{scs_error}")
                  except Exception as e_torque_disable:
                       self.get_logger().warn(f"Exception disabling torque for {name}(ID:{motor_id}): {e_torque_disable}")
                  time.sleep(0.02)
        
        self.cleanup_resources()
        super().destroy_node()
        self.get_logger().info(f"{NODE_NAME} node shut down complete.")

def main(args=None):
    rclpy.init(args=args)
    node = None
    logger = rclpy.logging.get_logger(f'{NODE_NAME}_main')
    try:
        node = RealRobotInterfaceNode()
        if not (node.port_handler and node.packet_handler and node.calibration_data and \
                node.group_writer and node.group_reader and node.control_signal_subscription and \
                node.joint_state_publisher and node.reader_motors_ok):
             logger.fatal("Node object created, but critical components are missing/uninitialized.")
             raise RuntimeError("Node initialization incomplete.")
        
        logger.info("RealRobotInterfaceNode initialization successful, starting spin loop.")
        rclpy.spin(node)
    except KeyboardInterrupt:
         logger.info('Ctrl+C detected, initiating shutdown.')
    except RuntimeError as e_runtime:
         logger.fatal(f"Caught RuntimeError during node setup or spin: {e_runtime}")
         # traceback.print_exc() # Already printed if it originated in __init__
    except Exception as e_unhandled:
        logger.fatal(f'Unhandled exception in main execution: {e_unhandled}')
        traceback.print_exc()
    finally:
        if node is not None:
            logger.info("Executing final node cleanup from main...")
            node.destroy_node()
        else:
            logger.warn("Node object was None or not fully created, skipping destroy_node call from main.")
        
        if rclpy.ok():
            rclpy.shutdown()
            logger.info("RCLPY shutdown complete.")
        print(f"{NODE_NAME} main process finished.")

if __name__ == '__main__':
    main()