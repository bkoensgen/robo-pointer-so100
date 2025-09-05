import math
from types import SimpleNamespace

import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from robo_pointer_visual.mock_robot_interface_node import MockRobotInterfaceNode


class _CapPub:
    def __init__(self):
        self.last = None

    def publish(self, msg):
        self.last = msg


def _js(names, degrees):
    js = JointState()
    js.name = names
    js.position = [math.radians(d) for d in degrees]
    return js


def test_mock_single_tick_rate_limit():
    rclpy.init()
    node = MockRobotInterfaceNode()
    try:
        # Stop periodic timer; we'll tick manually
        node.timer.cancel()
        cap = _CapPub()
        node.js_pub = cap

        # Set known speeds
        node.set_parameters([
            Parameter('sim_speed_deg_s.pan', Parameter.Type.DOUBLE, 60.0),
            Parameter('sim_speed_deg_s.lift', Parameter.Type.DOUBLE, 60.0),
            Parameter('sim_speed_deg_s.elbow', Parameter.Type.DOUBLE, 60.0),
            Parameter('sim_speed_deg_s.wrist', Parameter.Type.DOUBLE, 60.0),
        ])

        names = list(node.current_deg.keys())
        # Target far from current 0 deg
        tgt_deg = [90.0, -90.0, 45.0, -30.0]
        node._on_target(_js(names, tgt_deg))

        # Force dt = 1.0 s
        now_ns = node.get_clock().now().nanoseconds
        node._last_tick = SimpleNamespace(nanoseconds=now_ns - int(1e9))

        node._tick()

        assert cap.last is not None
        cmd_deg = [math.degrees(v) for v in cap.last.position]
        # Each joint should have moved by at most 60 deg toward target
        for i, n in enumerate(names):
            speed = node.max_speed[n]
            expected = max(-speed, min(speed, tgt_deg[i] - 0.0))
            assert math.isclose(cmd_deg[i], expected, abs_tol=1e-3)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_mock_converges_after_multiple_ticks():
    rclpy.init()
    node = MockRobotInterfaceNode()
    try:
        node.timer.cancel()
        cap = _CapPub()
        node.js_pub = cap

        node.set_parameters([
            Parameter('sim_speed_deg_s.pan', Parameter.Type.DOUBLE, 90.0),
            Parameter('sim_speed_deg_s.lift', Parameter.Type.DOUBLE, 90.0),
            Parameter('sim_speed_deg_s.elbow', Parameter.Type.DOUBLE, 90.0),
            Parameter('sim_speed_deg_s.wrist', Parameter.Type.DOUBLE, 90.0),
        ])

        names = list(node.current_deg.keys())
        tgt_deg = [45.0, 10.0, -20.0, 15.0]
        node._on_target(_js(names, tgt_deg))

        # Apply several 0.5s ticks
        for _ in range(6):
            now_ns = node.get_clock().now().nanoseconds
            node._last_tick = SimpleNamespace(nanoseconds=now_ns - int(5e8))
            node._tick()

        assert cap.last is not None
        cmd_deg = [math.degrees(v) for v in cap.last.position]
        # After enough ticks, should be very close to targets
        for i in range(len(names)):
            assert math.isclose(cmd_deg[i], tgt_deg[i], abs_tol=1.0), f"joint {names[i]} not close to target"
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_mock_dynamic_speed_update_changes_rate_limit():
    rclpy.init()
    node = MockRobotInterfaceNode()
    try:
        node.timer.cancel()
        cap = _CapPub()
        node.js_pub = cap

        names = list(node.current_deg.keys())
        tgt_deg = [90.0, 0.0, 0.0, 0.0]
        node._on_target(_js(names, tgt_deg))

        # Force dt = 1.0 s
        now_ns = node.get_clock().now().nanoseconds
        node._last_tick = SimpleNamespace(nanoseconds=now_ns - int(1e9))

        # Update pan speed to 30 deg/s dynamically
        node.set_parameters([Parameter('sim_speed_deg_s.pan', Parameter.Type.DOUBLE, 30.0)])
        node._tick()

        assert cap.last is not None
        cmd_deg = [math.degrees(v) for v in cap.last.position]
        # Pan should have moved by 30 degrees toward 90
        assert math.isclose(cmd_deg[0], 30.0, abs_tol=1e-3)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
