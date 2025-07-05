import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np
import math
import traceback

from .kinematics import (
    calculate_fk_wrist, calculate_ik, 
    calculate_gravity_compensation_adj_deg, 
    calculate_wrist_angle_for_horizontal,
    calculate_aim_offset
)

class RobotControllerNode(Node):
    """
    Nœud "Cerveau" du robot.
    - Reçoit les coordonnées d'une cible détectée par le nœud de vision.
    - Utilise la cinématique inverse pour calculer les angles des articulations nécessaires.
    - Applique des compensations (gravité, position de la caméra).
    - Publie les angles cibles vers l'interface matérielle du robot.
    """
    def __init__(self):
        super().__init__('robot_controller_node')
        self.get_logger().info('Robot Controller node has started.')

        # --- Paramètres de Contrôle ---
        self.declare_parameter('pixel_to_cartesian_scale_x', 0.0004)
        self.declare_parameter('pixel_to_cartesian_scale_y', 0.0004)
        self.declare_parameter('camera_tilt_angle_deg', 30.0)
        self.declare_parameter('camera_forward_offset_m', 0.05)
        self.declare_parameter('k_gravity_id2', 12.0)
        self.declare_parameter('k_gravity_id3', 6.0)
        self.declare_parameter('lift_angle_min_deg', 0.0)
        self.declare_parameter('lift_angle_max_deg', 130.0)
        self.declare_parameter('elbow_angle_min_deg', 0.0)
        self.declare_parameter('elbow_angle_max_deg', 110.0)
        self.declare_parameter('pan_angle_min_deg', -110.0)
        self.declare_parameter('pan_angle_max_deg', 110.0)
        self.declare_parameter('wrist_angle_min_deg', -100.0)
        self.declare_parameter('wrist_angle_max_deg', 100.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        # Récupération des paramètres
        self.scale_x = self.get_parameter('pixel_to_cartesian_scale_x').get_parameter_value().double_value
        self.scale_y = self.get_parameter('pixel_to_cartesian_scale_y').get_parameter_value().double_value
        self.camera_tilt_deg = self.get_parameter('camera_tilt_angle_deg').get_parameter_value().double_value
        self.camera_offset_m = self.get_parameter('camera_forward_offset_m').get_parameter_value().double_value
        self.k_gravity_id2 = self.get_parameter('k_gravity_id2').get_parameter_value().double_value
        self.k_gravity_id3 = self.get_parameter('k_gravity_id3').get_parameter_value().double_value
        self.lift_min_deg = self.get_parameter('lift_angle_min_deg').get_parameter_value().double_value
        self.lift_max_deg = self.get_parameter('lift_angle_max_deg').get_parameter_value().double_value
        self.elbow_min_deg = self.get_parameter('elbow_angle_min_deg').get_parameter_value().double_value
        self.elbow_max_deg = self.get_parameter('elbow_angle_max_deg').get_parameter_value().double_value
        self.pan_min_deg = self.get_parameter('pan_angle_min_deg').get_parameter_value().double_value
        self.pan_max_deg = self.get_parameter('pan_angle_max_deg').get_parameter_value().double_value
        self.wrist_min_deg = self.get_parameter('wrist_angle_min_deg').get_parameter_value().double_value
        self.wrist_max_deg = self.get_parameter('wrist_angle_max_deg').get_parameter_value().double_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        
        self.current_joint_states = None
        self.pan_motor_name = "shoulder_pan"
        self.lift_motor_name = "shoulder_lift"
        self.elbow_motor_name = "elbow_flex"
        self.wrist_motor_name = "wrist_flex"
        
        initial_lift, initial_elbow = 107.0, 85.0
        initial_wrist = calculate_wrist_angle_for_horizontal(initial_lift, initial_elbow)
        self.initial_pose_deg = {
            self.pan_motor_name: 0.0, self.lift_motor_name: initial_lift,
            self.elbow_motor_name: initial_elbow, self.wrist_motor_name: initial_wrist
        }
        self.target_subscription = self.create_subscription(Point, '/detected_target_point', self.target_callback, 10)
        self.joint_state_subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.target_angles_publisher = self.create_publisher(JointState, '/target_joint_angles', 10)
        
        # Timer pour envoyer la position de départ de manière robuste
        self.initial_pose_timer = self.create_timer(0.5, self.send_initial_pose_when_ready)

        self.get_logger().info('Ready. Waiting for subscriber to be ready...')

    def send_initial_pose_when_ready(self):
        """
        Vérifie si un abonné (le robot) est prêt avant d'envoyer la position
        de départ. S'annule après le premier envoi réussi.
        """
        if self.target_angles_publisher.get_subscription_count() > 0:
            self.get_logger().info(f"Subscriber ready. Sending initial pose: { {k: round(v, 1) for k, v in self.initial_pose_deg.items()} }")
            
            target_msg = JointState()
            target_msg.header.stamp = self.get_clock().now().to_msg()
            target_msg.name = list(self.initial_pose_deg.keys())
            target_msg.position = [math.radians(angle) for angle in self.initial_pose_deg.values()]
            
            self.target_angles_publisher.publish(target_msg)
            
            self.initial_pose_timer.cancel()
            self.get_logger().info("Initial pose sent. Controller is now active.")
        else:
            self.get_logger().warn("Waiting for real_robot_interface to subscribe...", throttle_duration_sec=2)

    def joint_state_callback(self, msg: JointState):
        """Met à jour l'état actuel des articulations du robot."""
        self.current_joint_states = msg

    def target_callback(self, msg: Point):
        """Callback principal pour le traitement d'une nouvelle cible."""
        if not self.initial_pose_timer.is_canceled(): return
        if msg.x == -1.0: return
        if self.current_joint_states is None: return

        try:
            joint_map = {name: math.degrees(pos) for name, pos in zip(self.current_joint_states.name, self.current_joint_states.position)}
            current_pan_deg = joint_map.get(self.pan_motor_name, self.initial_pose_deg[self.pan_motor_name])
            current_lift_deg = joint_map.get(self.lift_motor_name, self.initial_pose_deg[self.lift_motor_name])
            current_elbow_deg = joint_map.get(self.elbow_motor_name, self.initial_pose_deg[self.elbow_motor_name])

            current_xw, current_yw = calculate_fk_wrist(current_lift_deg, current_elbow_deg)
            
            error_x = msg.x - (self.image_width / 2.0)
            error_y = msg.y - (self.image_height / 2.0)

            target_pan_deg = current_pan_deg - (error_x * self.scale_x)

            # L'axe Y de l'image est inversé par rapport à l'axe Y cartésien du robot
            delta_yw = -error_y * self.scale_y
            perceived_target_yw = current_yw + delta_yw
            perceived_target_xw = current_xw

            # Compenser le décalage de la caméra pour trouver la cible IK réelle
            final_arm_orientation_deg = -(current_lift_deg + current_elbow_deg)
            offset_x, offset_y = calculate_aim_offset(final_arm_orientation_deg, self.camera_tilt_deg, self.camera_offset_m)
            ik_target_xw = perceived_target_xw - offset_x
            ik_target_yw = perceived_target_yw - offset_y
            
            ik_lift_rad, ik_elbow_rad, ik_ok = calculate_ik(ik_target_xw, ik_target_yw)
            if not ik_ok: return

            ideal_lift_deg = math.degrees(ik_lift_rad)
            ideal_elbow_deg = math.degrees(ik_elbow_rad)

            # Ajouter la compensation de gravité
            gravity_lift_adj, gravity_elbow_adj = calculate_gravity_compensation_adj_deg(
                ideal_lift_deg, ideal_elbow_deg, self.k_gravity_id2, self.k_gravity_id3
            )
            compensated_lift_deg = ideal_lift_deg + gravity_lift_adj
            compensated_elbow_deg = ideal_elbow_deg + gravity_elbow_adj

            target_wrist_deg = calculate_wrist_angle_for_horizontal(compensated_lift_deg, compensated_elbow_deg)

            # Limiter les angles finaux aux bornes physiques
            final_pan_deg = np.clip(target_pan_deg, self.pan_min_deg, self.pan_max_deg)
            final_lift_deg = np.clip(compensated_lift_deg, self.lift_min_deg, self.lift_max_deg)
            final_elbow_deg = np.clip(compensated_elbow_deg, self.elbow_min_deg, self.elbow_max_deg)
            final_wrist_deg = np.clip(target_wrist_deg, self.wrist_min_deg, self.wrist_max_deg)
            
            target_joint_state_msg = JointState()
            target_joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            target_joint_state_msg.name = [self.pan_motor_name, self.lift_motor_name, self.elbow_motor_name, self.wrist_motor_name]
            target_joint_state_msg.position = [
                math.radians(final_pan_deg), math.radians(final_lift_deg),
                math.radians(final_elbow_deg), math.radians(final_wrist_deg)
            ]
            
            self.target_angles_publisher.publish(target_joint_state_msg)

        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")
            traceback.print_exc()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RobotControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
