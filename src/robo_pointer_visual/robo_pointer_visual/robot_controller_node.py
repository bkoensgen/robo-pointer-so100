# Fichier: ~/ros2_ws/src/robo_pointer_visual/robo_pointer_visual/robot_controller_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point  # Pour recevoir les coordonnées
from std_msgs.msg import String      # Pour publier la commande textuelle

# Supprimé les constantes globales IMAGE_WIDTH, IMAGE_HEIGHT

class RobotControllerNode(Node):
    """
    Ce nœud s'abonne aux coordonnées de la cible détectée (/detected_target_point),
    interprète la position de la cible dans l'image (en utilisant des paramètres ROS 2
    pour la taille de l'image et une zone morte centrale), détermine une commande
    directionnelle simple, et publie cette commande sur /robot_command.
    """
    def __init__(self):
        # Initialise le nœud ROS 2
        super().__init__('robot_controller_node')
        self.get_logger().info('Robot Controller node has started.')

        # --- Paramètres ROS 2 ---
        self.declare_parameter('image_width', 640) # Valeur par défaut 640
        self.declare_parameter('image_height', 480) # Valeur par défaut 480
        self.declare_parameter('center_deadzone_ratio_x', 0.2) # 20% de la largeur comme zone morte
        self.declare_parameter('center_deadzone_ratio_y', 0.2) # 20% de la hauteur comme zone morte

        # Récupérer les valeurs des paramètres
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        deadzone_ratio_x = self.get_parameter('center_deadzone_ratio_x').get_parameter_value().double_value
        deadzone_ratio_y = self.get_parameter('center_deadzone_ratio_y').get_parameter_value().double_value

        # Calculer les seuils absolus en pixels pour la zone morte (la moitié de la zone morte totale)
        self.deadzone_threshold_x = (self.image_width * deadzone_ratio_x) / 2.0
        self.deadzone_threshold_y = (self.image_height * deadzone_ratio_y) / 2.0

        self.get_logger().info(f'Using image dimensions: {self.image_width}x{self.image_height}')
        self.get_logger().info(f'Center deadzone thresholds: +/-{self.deadzone_threshold_x:.0f}px X, +/-{self.deadzone_threshold_y:.0f}px Y')

        # --- Subscriber aux coordonnées de la cible ---
        self.target_subscription = self.create_subscription(
            Point,
            '/detected_target_point',
            self.target_callback,
            10 # QoS depth
        )
        self.get_logger().info('Subscribed to /detected_target_point')

        # --- Publisher pour la commande robot ---
        self.command_publisher = self.create_publisher(
            String,
            '/robot_command',
            10 # QoS depth
        )
        self.get_logger().info('Publishing robot commands on /robot_command')

        # --- Variable pour état interne ---
        self.last_command = None # Mémorise la dernière commande publiée

    def target_callback(self, msg):
        """
        Callback appelé à chaque message Point reçu.
        Interprète les coordonnées en utilisant les zones et la zone morte,
        et publie une commande si elle change.
        """
        received_x = msg.x
        received_y = msg.y

        current_command = None # Commande déterminée pour cette trame

        # --- 1. Interpréter les Coordonnées reçues ---
        if received_x == -1.0:
            # Aucune cible valide détectée par le nœud de vision
            current_command = "IDLE"
        else:
            # Cible détectée, vérifier d'abord la zone morte centrale
            center_x = self.image_width / 2.0
            center_y = self.image_height / 2.0

            if abs(received_x - center_x) < self.deadzone_threshold_x and \
               abs(received_y - center_y) < self.deadzone_threshold_y:
                 # La cible est dans la zone morte centrale
                 current_command = "CENTERED"
            else:
                 # La cible est en dehors de la zone morte, déterminer la zone externe
                 left_bound = self.image_width / 3
                 right_bound = 2 * self.image_width / 3
                 bottom_bound = self.image_height / 3 # Rappel: Y=0 est en HAUT
                 top_bound = 2 * self.image_height / 3

                 # Logique de décision basée sur 8 zones externes
                 if received_y < bottom_bound: # Zone du bas de l'image
                      if received_x < left_bound:
                           current_command = "MOVE_BOTTOM_LEFT"
                      elif received_x > right_bound:
                           current_command = "MOVE_BOTTOM_RIGHT"
                      else:
                           current_command = "MOVE_BOTTOM"
                 elif received_y > top_bound: # Zone du haut de l'image
                      if received_x < left_bound:
                           current_command = "MOVE_TOP_LEFT"
                      elif received_x > right_bound:
                           current_command = "MOVE_TOP_RIGHT"
                      else:
                           current_command = "MOVE_TOP"
                 else: # Zone du milieu (verticalement), hors zone morte centrale
                      if received_x < left_bound:
                           current_command = "MOVE_LEFT"
                      elif received_x > right_bound:
                           current_command = "MOVE_RIGHT"
                      # Pas de 'else' ici, le cas central est déjà traité

        # --- 2. Publier la commande SEULEMENT si elle a changé ---
        if current_command != self.last_command:
            # Logger la nouvelle commande
            if current_command == "IDLE":
                self.get_logger().info('No target detected. Command: IDLE')
            elif current_command == "CENTERED":
                self.get_logger().info(f'Target is centered. Command: {current_command}')
            else:
                # Utiliser WARN pour voir facilement les demandes de mouvement
                self.get_logger().warn(f'Target at ({received_x:.0f}, {received_y:.0f}). Command: {current_command}')

            # Créer le message String et le publier
            command_msg = String()
            command_msg.data = current_command if current_command is not None else "UNKNOWN"
            self.command_publisher.publish(command_msg)

            # Mettre à jour la dernière commande publiée
            self.last_command = current_command


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
