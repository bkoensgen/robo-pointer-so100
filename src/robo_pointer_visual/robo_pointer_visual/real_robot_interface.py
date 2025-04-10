# Fichier: ~/ros2_ws/src/robo_pointer_visual/robo_pointer_visual/real_robot_interface.py

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

# Assurez-vous que le chemin d'import est correct pour votre installation lerobot
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus, FeetechMotorsBusConfig

LEADER_ARM_PORT = "/dev/ttyACM0" # Vérifiez ce port

LEADER_ARM_MOTORS = {
    "shoulder_pan": [1, "sts3215"],
    "shoulder_lift": [2, "sts3215"],
    "elbow_flex": [3, "sts3215"],
    "wrist_flex": [4, "sts3215"],
    "wrist_roll": [5, "sts3215"],
    "gripper": [6, "sts3215"],
}

# Basic calibration profile (assuming STS3215 resolution 4096)
# Homing offset = step value corresponding to 0 degrees
# Drive mode 0 = non-inverted
DEFAULT_CALIBRATION = {
    "motor_names": ["shoulder_pan", "shoulder_lift"],
    "calib_mode": ["DEGREE", "DEGREE"],
    "drive_mode": [0, 0], # Adjust if needed based on physical rotation direction
    "homing_offset": [2048, 2048], # Needs verification/fine-tuning
    "start_pos": [0, 0], # Not used for DEGREE mode
    "end_pos": [0, 0] # Not used for DEGREE mode
}


class RealRobotInterfaceNode(Node):
    """
    Interfaces with the real Feetech robot arm.
    Subscribes to /robot_control_signal (Vector3).
    Reads current position, calculates target position based on signal,
    and writes Goal_Position to motors using FeetechMotorsBus with calibration.
    """
    def __init__(self):
        super().__init__('real_robot_interface')
        self.get_logger().info('Real Robot Interface node starting...')

        self.leader_bus = None
        leader_config = FeetechMotorsBusConfig(port=LEADER_ARM_PORT, motors=LEADER_ARM_MOTORS)
        self.leader_bus = FeetechMotorsBus(leader_config)

        try:
            self.leader_bus.connect()
            self.get_logger().info(f"Successfully connected to leader arm bus on {LEADER_ARM_PORT}")
            # Set the calibration profile for the bus
            self.leader_bus.set_calibration(DEFAULT_CALIBRATION)
            self.get_logger().info("Default calibration profile set for the motors bus.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect/setup leader arm bus on {LEADER_ARM_PORT}: {e}")
            self.leader_bus = None
            # Optional: raise an error or shutdown if connection fails
            # raise e

        # Parameters for control logic
        self.declare_parameter('angle_increment_scale', 0.1) # Scales control signal to angle delta
        self.angle_increment_scale = self.get_parameter('angle_increment_scale').get_parameter_value().double_value
        self.declare_parameter('stop_threshold', 0.005) # Threshold below which signal is considered zero
        self.stop_threshold = self.get_parameter('stop_threshold').get_parameter_value().double_value

        # Parameters for joint limits (in degrees) - VERIFY THESE VALUES
        self.declare_parameter('pan_angle_min', -150.0)
        self.declare_parameter('pan_angle_max', 150.0)
        self.declare_parameter('lift_angle_min', -90.0)
        self.declare_parameter('lift_angle_max', 90.0)

        self.pan_min = self.get_parameter('pan_angle_min').get_parameter_value().double_value
        self.pan_max = self.get_parameter('pan_angle_max').get_parameter_value().double_value
        self.lift_min = self.get_parameter('lift_angle_min').get_parameter_value().double_value
        self.lift_max = self.get_parameter('lift_angle_max').get_parameter_value().double_value

        self.get_logger().info(f"Control params: Scale={self.angle_increment_scale}, StopThr={self.stop_threshold}")
        self.get_logger().info(f'Limits (degrees): Pan[{self.pan_min},{self.pan_max}], Lift[{self.lift_min},{self.lift_max}]')

        self.control_signal_subscription = self.create_subscription(
            Vector3,
            '/robot_control_signal',
            self.control_signal_callback,
            10
        )
        self.get_logger().info('Subscribed to /robot_control_signal')

    def control_signal_callback(self, msg):
        if self.leader_bus is None or not self.leader_bus.is_connected:
            self.get_logger().warn("Leader bus not connected, skipping command.", throttle_duration_sec=5)
            return

        signal_pan = msg.x
        signal_lift = msg.y

        self.get_logger().debug(f'Received Signal: [Pan={signal_pan:.2f}, Lift={signal_lift:.2f}]')

        delta_angle_pan = self.angle_increment_scale * signal_pan
        delta_angle_lift = self.angle_increment_scale * signal_lift

        # --- Shoulder Pan Control (ID 1) ---
        if abs(delta_angle_pan) >= self.stop_threshold:
            try:
                motor_name_pan = "shoulder_pan"
                # Read returns degrees now due to calibration
                current_pan_raw = self.leader_bus.read("Present_Position", motor_name_pan)
                current_pan = current_pan_raw.item()
                self.get_logger().debug(f"Read current_pan (deg): {current_pan_raw} -> Extracted: {current_pan}")

                target_pan = current_pan + delta_angle_pan
                target_pan_clipped = np.clip(target_pan, self.pan_min, self.pan_max)

                if abs(target_pan - target_pan_clipped) > 0.1:
                    self.get_logger().warn(f"Pan target {target_pan:.1f} clipped to {target_pan_clipped:.1f}")

                self.get_logger().info(f"PAN: Curr={current_pan:.1f}, Delta={delta_angle_pan:.2f} -> Target={target_pan_clipped:.1f}")
                # Write expects degrees now due to calibration
                self.leader_bus.write("Goal_Position", target_pan_clipped, motor_name_pan)

            except Exception as e:
                self.get_logger().error(f"Failed to command {motor_name_pan}: {e}")
        else:
             self.get_logger().debug(f"Pan signal {signal_pan:.2f} below threshold, holding.")


        # --- Shoulder Lift Control (ID 2) ---
        if abs(delta_angle_lift) >= self.stop_threshold:
            try:
                motor_name_lift = "shoulder_lift"
                # Read returns degrees now due to calibration
                current_lift_raw = self.leader_bus.read("Present_Position", motor_name_lift)
                current_lift = current_lift_raw.item()
                self.get_logger().debug(f"Read current_lift (deg): {current_lift_raw} -> Extracted: {current_lift}")

                target_lift = current_lift + delta_angle_lift
                target_lift_clipped = np.clip(target_lift, self.lift_min, self.lift_max)

                if abs(target_lift - target_lift_clipped) > 0.1:
                    self.get_logger().warn(f"Lift target {target_lift:.1f} clipped to {target_lift_clipped:.1f}")

                self.get_logger().info(f"LIFT: Curr={current_lift:.1f}, Delta={delta_angle_lift:.2f} -> Target={target_lift_clipped:.1f}")
                # Write expects degrees now due to calibration
                self.leader_bus.write("Goal_Position", target_lift_clipped, motor_name_lift)

            except Exception as e:
                self.get_logger().error(f"Failed to command {motor_name_lift}: {e}")
        else:
             self.get_logger().debug(f"Lift signal {signal_lift:.2f} below threshold, holding.")


    def destroy_node(self):
        self.get_logger().info('Shutting down Real Robot Interface node...')
        if self.leader_bus is not None and self.leader_bus.is_connected:
            try:
                # Optional: consider commanding motors to a safe resting position or disabling torque here
                # self.leader_bus.write("Torque_Enable", 0) # Example: Disable torque
                self.leader_bus.disconnect()
                self.get_logger().info("Leader bus disconnected.")
            except Exception as e:
                self.get_logger().error(f"Error during leader bus cleanup: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    real_robot_interface_node = None
    try:
        real_robot_interface_node = RealRobotInterfaceNode()
        # Only spin if initialization (including bus connection) was successful
        if real_robot_interface_node.leader_bus is not None and real_robot_interface_node.leader_bus.is_connected:
            rclpy.spin(real_robot_interface_node)
        else:
            real_robot_interface_node.get_logger().fatal("Initialization failed (bus not connected). Shutting down.")

    except KeyboardInterrupt:
        if real_robot_interface_node:
            real_robot_interface_node.get_logger().info('Ctrl+C detected, shutting down interface node.')
    except Exception as e:
        # Log any unexpected error during spin or init
        if real_robot_interface_node:
            real_robot_interface_node.get_logger().fatal(f'Unhandled exception: {e}')
        else:
            print(f'Unhandled exception during node creation: {e}')
    finally:
        # Ensure cleanup happens
        if real_robot_interface_node is not None:
            real_robot_interface_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            print("Interface node shutdown complete.")

if __name__ == '__main__':
    main()
