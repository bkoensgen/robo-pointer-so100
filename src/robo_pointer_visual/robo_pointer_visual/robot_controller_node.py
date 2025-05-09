# Fichier: ~/ros2_ws/src/robo_pointer_visual/robo_pointer_visual/robot_controller_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Point

class RobotControllerNode(Node):
    """
    Ce nœud s'abonne aux coordonnées de la cible détectée (/detected_target_point),
    calcule l'erreur par rapport au centre, détermine un signal de contrôle proportionnel,
    et publie ce signal (Vector3) sur /robot_control_signal.
    """
    def __init__(self):
        # Initialise le nœud ROS 2
        super().__init__('robot_controller_node')
        self.get_logger().info('Robot Controller node has started.')

        # --- Paramètres ROS 2 ---
        self.declare_parameter('image_width', 640) # Valeur par défaut 640
        self.declare_parameter('image_height', 480) # Valeur par défaut 480

        # Nouveaux paramètres pour les gains proportionnels
        self.declare_parameter('kp_pan', 0.1) # Gain pour le mouvement horizontal (pan)
        self.declare_parameter('kp_lift', 0.2) # Gain pour le mouvement vertical (lift)

        # Récupérer les valeurs des paramètres
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.kp_pan = self.get_parameter('kp_pan').get_parameter_value().double_value
        self.kp_lift = self.get_parameter('kp_lift').get_parameter_value().double_value

        self.get_logger().info(f'Using image dimensions: {self.image_width}x{self.image_height}')
        self.get_logger().info(f'Using proportional gains: Kp_pan={self.kp_pan}, Kp_lift={self.kp_lift}')

        # --- Subscriber aux coordonnées de la cible ---
        self.target_subscription = self.create_subscription(
            Point,
            '/detected_target_point',
            self.target_callback,
            10 # QoS depth
        )
        self.get_logger().info('Subscribed to /detected_target_point')

        # --- Publisher pour le signal de contrôle robot ---
        self.control_signal_publisher = self.create_publisher(
            Vector3,
            '/robot_control_signal',
            10 # QoS depth
        )
        self.get_logger().info('Publishing control signals on /robot_control_signal')

    def target_callback(self, msg):
        """
        Callback appelé à chaque message Point reçu.
        Calcule l'erreur par rapport au centre et publie un signal de contrôle
        proportionnel (Vector3) sur /robot_control_signal.
        """
        cx = msg.x
        cy = msg.y

        control_signal = Vector3() # Initialisé à (0,0,0)

        if cx == -1.0:
            # Aucune cible détectée, envoyer un signal nul
            self.get_logger().info('No target detected. Publishing null control signal.')
        else:
            # Cible détectée, calculer l'erreur et la commande proportionnelle
            center_x = self.image_width / 2.0
            center_y = self.image_height / 2.0

            error_x = cx - center_x
            # Axe Y image : 0 en haut. Erreur positive = objet plus bas que le centre
            error_y = cy - center_y

            # Calcul des commandes proportionnelles
            command_pan = -self.kp_pan * error_x
            command_lift = -self.kp_lift * error_y

            control_signal.x = command_pan
            control_signal.y = command_lift
            control_signal.z = 0.0 # Non utilisé pour l'instant

            # Logger les infos pour le débogage
            self.get_logger().info(f'Target at ({cx:.0f}, {cy:.0f}) -> Error(x={error_x:.1f}, y={error_y:.1f}) -> Signal(x={command_pan:.2f}, y={command_lift:.2f})')

        # Toujours publier le signal de contrôle (calculé ou nul)
        self.control_signal_publisher.publish(control_signal)


def main(args=None):
    rclpy.init(args=args)
    robot_controller_node = None
    try:
        robot_controller_node = RobotControllerNode()
        rclpy.spin(robot_controller_node)
    except KeyboardInterrupt:
        if robot_controller_node:
             robot_controller_node.get_logger().info('Ctrl+C detected, shutting down controller node.')
        else:
             print('Ctrl+C detected during node initialization.')
    except Exception as e:
        if robot_controller_node:
            robot_controller_node.get_logger().error(f'Controller node error: {e}')
        else:
            print(f'Error creating controller node: {e}')
    finally:
        if robot_controller_node is not None:
            robot_controller_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            print("Controller node shutdown complete.")

if __name__ == '__main__':
    main()
