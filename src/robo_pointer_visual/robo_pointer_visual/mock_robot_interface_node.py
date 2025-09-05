import math
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import SetParametersResult


DEFAULT_MOTOR_ORDER: List[str] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
]


class MockRobotInterfaceNode(Node):
    """
    Mock interface that follows target_joint_angles with per-joint speed limits
    and publishes joint_states at a fixed rate. Useful for offline demos/tuning.
    """

    def __init__(self) -> None:
        super().__init__("mock_robot_interface")

        # Parameters
        self.declare_parameter("read_frequency_hz", 20.0)
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("target_joint_angles_topic", "target_joint_angles")
        # Per-joint simulated max speeds (deg/s)
        self.declare_parameter("sim_speed_deg_s.pan", 90.0)
        self.declare_parameter("sim_speed_deg_s.lift", 60.0)
        self.declare_parameter("sim_speed_deg_s.elbow", 60.0)
        self.declare_parameter("sim_speed_deg_s.wrist", 120.0)

        freq = float(self.get_parameter("read_frequency_hz").get_parameter_value().double_value)
        self.js_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        self.tgt_topic = self.get_parameter("target_joint_angles_topic").get_parameter_value().string_value
        self.max_speed: Dict[str, float] = {
            "shoulder_pan": float(self.get_parameter("sim_speed_deg_s.pan").value),
            "shoulder_lift": float(self.get_parameter("sim_speed_deg_s.lift").value),
            "elbow_flex": float(self.get_parameter("sim_speed_deg_s.elbow").value),
            "wrist_flex": float(self.get_parameter("sim_speed_deg_s.wrist").value),
        }

        # State
        self.current_deg: Dict[str, float] = {name: 0.0 for name in DEFAULT_MOTOR_ORDER}
        self.target_deg: Dict[str, float] = dict(self.current_deg)
        self._last_tick = self.get_clock().now()

        # ROS I/O
        self.js_pub = self.create_publisher(JointState, self.js_topic, 10)
        self.tgt_sub = self.create_subscription(JointState, self.tgt_topic, self._on_target, 10)
        self.timer = self.create_timer(1.0 / max(freq, 1e-3), self._tick)

        self.get_logger().info(
            f"Mock interface running at {freq} Hz. Speeds (deg/s): "
            f"pan={self.max_speed['shoulder_pan']}, lift={self.max_speed['shoulder_lift']}, "
            f"elbow={self.max_speed['elbow_flex']}, wrist={self.max_speed['wrist_flex']}"
        )
        # Dynamic parameter updates for speeds
        self.add_on_set_parameters_callback(self._on_set_parameters)

    def _on_target(self, msg: JointState) -> None:
        # Map incoming joints to our tracked set
        for name, pos in zip(msg.name, msg.position):
            if name in self.target_deg:
                self.target_deg[name] = math.degrees(pos)

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now.nanoseconds - self._last_tick.nanoseconds) * 1e-9
        if dt <= 0.0:
            dt = 1e-3
        self._last_tick = now

        # Integrate towards target with speed limits
        for name in DEFAULT_MOTOR_ORDER:
            cur = self.current_deg[name]
            tgt = self.target_deg[name]
            max_delta = self.max_speed[name] * dt
            delta = max(-max_delta, min(max_delta, tgt - cur))
            self.current_deg[name] = cur + delta

        # Publish joint_states
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = list(DEFAULT_MOTOR_ORDER)
        js.position = [math.radians(self.current_deg[n]) for n in DEFAULT_MOTOR_ORDER]
        self.js_pub.publish(js)

    def _on_set_parameters(self, params):
        # Allow dynamic update of simulated speeds
        for p in params:
            n = p.name
            if n == 'sim_speed_deg_s.pan':
                try:
                    v = float(p.value)
                    if v <= 0.0:
                        return SetParametersResult(successful=False, reason='sim_speed_deg_s.pan must be > 0')
                    self.max_speed['shoulder_pan'] = v
                except Exception:
                    return SetParametersResult(successful=False, reason='invalid pan speed')
            elif n == 'sim_speed_deg_s.lift':
                try:
                    v = float(p.value);  assert v > 0.0
                    self.max_speed['shoulder_lift'] = v
                except Exception:
                    return SetParametersResult(successful=False, reason='invalid lift speed')
            elif n == 'sim_speed_deg_s.elbow':
                try:
                    v = float(p.value);  assert v > 0.0
                    self.max_speed['elbow_flex'] = v
                except Exception:
                    return SetParametersResult(successful=False, reason='invalid elbow speed')
            elif n == 'sim_speed_deg_s.wrist':
                try:
                    v = float(p.value);  assert v > 0.0
                    self.max_speed['wrist_flex'] = v
                except Exception:
                    return SetParametersResult(successful=False, reason='invalid wrist speed')
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = MockRobotInterfaceNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor:
            executor.shutdown()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
