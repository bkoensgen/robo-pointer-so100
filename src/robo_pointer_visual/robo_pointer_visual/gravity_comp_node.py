# gravity_comp_node.py

import rclpy
from rclpy.node import node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

from kinematics import jacobian_translational

class GravityCompNode(Node):
    def __init__(self):
        super().__init__('gravity_comp_node')

        # --- ROS Parameters ---
        self.masses = [2.0,1.5, 1.0]
        
