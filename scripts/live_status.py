#!/usr/bin/env python3
"""
Live status dashboard (one terminal) for the mock/real pipeline.

Shows per-second rates for core topics and the latest detection status/values.

Usage:
  source scripts/env.sh
  python3 scripts/live_status.py

Stop with Ctrl+C.
"""

import time
import sys
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, JointState


@dataclass
class TopicStats:
    count: int = 0
    last_count: int = 0
    last_rate: float = 0.0


class LiveStatus(Node):
    def __init__(self):
        super().__init__('live_status')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        self.stats = {
            'image_debug': TopicStats(),
            'detected_target_point': TopicStats(),
            'target_joint_angles': TopicStats(),
            'joint_states': TopicStats(),
            'detection_acquired': TopicStats(),
            'detection_area': TopicStats(),
        }

        self.last_point = Point()
        self.acquired = False
        self.area = 0.0
        self.last_target_joint_names = []
        self.last_target_joint_deg = []

        self.create_subscription(Image, 'image_debug', self._on_image, qos)
        self.create_subscription(Point, 'detected_target_point', self._on_point, qos)
        self.create_subscription(JointState, 'target_joint_angles', self._on_target_js, qos)
        self.create_subscription(JointState, 'joint_states', self._on_js, qos)
        self.create_subscription(Bool, 'detection_acquired', self._on_acq, qos)
        self.create_subscription(Float32, 'detection_area', self._on_area, qos)

        self.timer = self.create_timer(1.0, self._tick)
        self.started = time.time()

    def _bump(self, key: str):
        if key in self.stats:
            self.stats[key].count += 1

    def _on_image(self, _msg: Image):
        self._bump('image_debug')

    def _on_point(self, msg: Point):
        self._bump('detected_target_point')
        self.last_point = msg

    def _on_target_js(self, msg: JointState):
        self._bump('target_joint_angles')
        self.last_target_joint_names = list(msg.name)
        try:
            self.last_target_joint_deg = [v * 180.0 / 3.141592653589793 for v in msg.position]
        except Exception:
            self.last_target_joint_deg = []

    def _on_js(self, _msg: JointState):
        self._bump('joint_states')

    def _on_acq(self, msg: Bool):
        self._bump('detection_acquired')
        self.acquired = bool(msg.data)

    def _on_area(self, msg: Float32):
        self._bump('detection_area')
        self.area = float(msg.data)

    def _rate_line(self, key: str, label: str) -> str:
        st = self.stats[key]
        delta = st.count - st.last_count
        st.last_count = st.count
        st.last_rate = delta  # since we print once per second
        return f"{label:<22} {st.last_rate:6.2f} Hz"

    def _pubs(self, topic: str) -> int:
        try:
            infos = self.get_publishers_info_by_topic(topic)
            return len(infos)
        except Exception:
            return 0

    def _tick(self):
        now = time.time()
        uptime = now - self.started
        try:
            # Clear-like header (keep it simple without ANSI control codes)
            print("\n" * 2, end="")
            print("Live Status (1s refresh) — uptime: %.0fs" % uptime)
            print("Topics present — pubs: image_debug=%d, target=%d, target_angles=%d, joint_states=%d" % (
                self._pubs('image_debug'),
                self._pubs('detected_target_point'),
                self._pubs('target_joint_angles'),
                self._pubs('joint_states'),
            ))
            print(self._rate_line('image_debug', 'image_debug'))
            print(self._rate_line('detected_target_point', 'detected_target_point'))
            print(self._rate_line('target_joint_angles', 'target_joint_angles'))
            print(self._rate_line('joint_states', 'joint_states'))
            print(self._rate_line('detection_acquired', 'detection_acquired'))
            print(self._rate_line('detection_area', 'detection_area'))

            lp = self.last_point
            print("Detection: acquired=%s  area=%.1f  point=(%.1f, %.1f)" % (
                str(self.acquired), self.area, getattr(lp, 'x', -1.0), getattr(lp, 'y', -1.0)))

            if self.last_target_joint_names and self.last_target_joint_deg:
                pairs = [f"{n}={d:.1f}°" for n, d in zip(self.last_target_joint_names, self.last_target_joint_deg)]
                print("Target angles: ", ", ".join(pairs))
            sys.stdout.flush()
        except Exception as e:
            # Keep running even if printing fails
            print(f"[status] print error: {e}")


def main():
    rclpy.init()
    node = LiveStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

