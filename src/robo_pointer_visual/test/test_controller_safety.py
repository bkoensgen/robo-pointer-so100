import math
import time

import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from robo_pointer_visual.robot_controller_node import RobotControllerNode


class _CapturePublisher:
    def __init__(self):
        self.last = None

    def publish(self, msg):
        self.last = msg


def _make_joint_state(names, degrees):
    js = JointState()
    js.name = names
    js.position = [math.radians(d) for d in degrees]
    return js


def test_dead_zone_suppresses_pan_motion():
    rclpy.init()
    node = RobotControllerNode()
    try:
        # Prevent initial pose sending logic from interfering
        node.initial_pose_timer.cancel()

        # Capture publishes from controller
        cap = _CapturePublisher()
        node.target_angles_publisher = cap

        # Configure dead zone
        node.set_parameters([Parameter('dead_zone_px', Parameter.Type.INTEGER, 12)])

        # Provide current joint states
        names = [node.pan_motor_name, node.lift_motor_name, node.elbow_motor_name, node.wrist_motor_name]
        current_deg = [10.0, 50.0, 30.0, -20.0]
        node.joint_state_callback(_make_joint_state(names, current_deg))

        # Target point very close to image center (within dead-zone on both axes)
        cx = node.image_width / 2.0
        cy = node.image_height / 2.0
        node.target_callback(Point(x=cx + 5.0, y=cy + 5.0, z=0.0))

        assert cap.last is not None, "Controller should publish a target JointState"
        # Pan should remain effectively unchanged (dead-zone cancels small error_x)
        pan_cmd_rad = cap.last.position[0]
        assert math.isclose(pan_cmd_rad, math.radians(current_deg[0]), abs_tol=1e-3)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_homing_moves_towards_initial_pose_with_rate_limit():
    rclpy.init()
    node = RobotControllerNode()
    try:
        node.initial_pose_timer.cancel()
        cap = _CapturePublisher()
        node.target_angles_publisher = cap

        # Set fast speeds for test and activate homing after short timeout
        node.set_parameters([
            Parameter('no_detection_behavior', Parameter.Type.STRING, 'home'),
            Parameter('no_detection_timeout_s', Parameter.Type.DOUBLE, 0.1),
            Parameter('joint_speed_max_deg_s.pan', Parameter.Type.DOUBLE, 90.0),
            Parameter('joint_speed_max_deg_s.lift', Parameter.Type.DOUBLE, 60.0),
            Parameter('joint_speed_max_deg_s.elbow', Parameter.Type.DOUBLE, 60.0),
            Parameter('joint_speed_max_deg_s.wrist', Parameter.Type.DOUBLE, 120.0),
        ])

        # Current far from initial to observe motion toward home
        names = [node.pan_motor_name, node.lift_motor_name, node.elbow_motor_name, node.wrist_motor_name]
        current_deg = [45.0, 20.0, 10.0, 0.0]
        node.joint_state_callback(_make_joint_state(names, current_deg))

        # Simulate last detection long ago to trigger homing
        now_ns = node.get_clock().now().nanoseconds
        node._last_detection_time_ns = now_ns - int(1.5e9)  # 1.5 s ago
        node._last_home_tick_ns = now_ns - int(1.0e9)       # dt ≈ 1.0 s

        node.homing_tick()

        assert cap.last is not None, "Homing should publish a target JointState"
        cmd_deg = [math.degrees(v) for v in cap.last.position]

        # For each joint, command should move towards initial_pose_deg and not exceed speed limit*dt
        dt = 1.0
        max_speeds = [node.max_speed_pan, node.max_speed_lift, node.max_speed_elbow, node.max_speed_wrist]
        initials = [node.initial_pose_deg[n] for n in names]
        for i in range(4):
            delta_before = initials[i] - current_deg[i]
            delta_after = cmd_deg[i] - current_deg[i]
            # Should move in the same direction towards initial
            assert math.copysign(1.0, delta_after) == math.copysign(1.0, delta_before) or math.isclose(delta_after, 0.0, abs_tol=1e-6)
            # Magnitude limited by speed*dt
            assert abs(delta_after) <= max_speeds[i] * dt + 1e-6
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

