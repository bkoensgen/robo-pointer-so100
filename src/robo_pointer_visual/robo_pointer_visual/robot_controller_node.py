# Fichier: ~/ros2_ws/src/robo_pointer_visual/robo_pointer_visual/robot_controller_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point  # Pour recevoir les coordonnées
from std_msgs.msg import String      # Pour publier la commande textuelle

# Dimensions supposées de l'image pour la logique de zones
# TODO: Rendre ces valeurs dynamiques ou configurables via paramètres
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

class RobotControllerNode(Node):
    """
    Ce nœud s'abonne aux coordonnées de la cible détectée (/detected_target_point),
    interprète la position de la cible dans l'image, détermine une commande
    directionnelle simple, et publie cette commande sur /robot_command.
    """
    def __init__(self):
        # Initialise le nœud ROS 2
        super().__init__('robot_controller_node')
        self.get_logger().info('Robot Controller node has started.')

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
        # Mémorise la dernière commande publiée pour éviter le spamming du topic
        self.last_command = None

    def target_callback(self, msg):
        """
        Callback appelé à chaque message Point reçu sur /detected_target_point.
        Interprète les coordonnées et publie une commande si elle change.
        """
        received_x = msg.x
        received_y = msg.y

        current_command = None # Commande déterminée pour cette trame

        # --- 1. Interpréter les Coordonnées reçues ---
        if received_x == -1.0:
            # Aucune cible valide détectée par le nœud de vision
            current_command = "IDLE"
        else:
            # Cible détectée, déterminer la zone de l'image où elle se trouve
            # Définition des limites pour les zones (basées sur des tiers de l'image)
            left_bound = IMAGE_WIDTH / 3
            right_bound = 2 * IMAGE_WIDTH / 3
            bottom_bound = IMAGE_HEIGHT / 3 # Rappel: Y=0 est en HAUT de l'image OpenCV
            top_bound = 2 * IMAGE_HEIGHT / 3

            # Logique de décision basée sur 9 zones (type morpion)
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
            else: # Zone du milieu (verticalement)
                if received_x < left_bound:
                    current_command = "MOVE_LEFT"
                elif received_x > right_bound:
                    current_command = "MOVE_RIGHT"
                else:
                    # La cible est considérée comme centrée
                    current_command = "CENTERED"

        # --- 2. Publier la commande SEULEMENT si elle a changé ---
        if current_command != self.last_command:
            # Logger la nouvelle commande (utile pour le débogage)
            if current_command == "IDLE":
                self.get_logger().info('No target detected. Command: IDLE')
            elif current_command == "CENTERED":
                self.get_logger().info(f'Target is centered. Command: {current_command}')
            else:
                # Mettre en WARN pour voir facilement les demandes de mouvement
                self.get_logger().warn(f'Target at ({received_x:.0f}, {received_y:.0f}). Command: {current_command}')

            # Créer le message String et le publier
            command_msg = String()
            # S'assurer qu'on a une string valide à publier
            command_msg.data = current_command if current_command is not None else "UNKNOWN"
            self.command_publisher.publish(command_msg)

            # Mettre à jour la dernière commande publiée
            self.last_command = current_command

        # Le log DEBUG des coordonnées brutes peut être enlevé ou commenté
        # self.get_logger().debug(f'Received target coordinates: x={received_x:.1f}, y={received_y:.1f}')


def main(args=None):
    rclpy.init(args=args)
    robot_controller_node = None # Bonne pratique d'initialiser à None
    try:
        robot_controller_node = RobotControllerNode()
        # spin() bloque l'exécution ici et gère les callbacks du subscriber
        rclpy.spin(robot_controller_node)
    except KeyboardInterrupt:
        # Gère le Ctrl+C proprement
        if robot_controller_node:
             robot_controller_node.get_logger().info('Ctrl+C detected, shutting down controller node.')
        else:
             print('Ctrl+C detected during node initialization.')
    except Exception as e:
        # Attrape d'autres erreurs potentielles
        if robot_controller_node:
            robot_controller_node.get_logger().error(f'Controller node error: {e}')
        else:
            print(f'Error creating controller node: {e}')
    finally:
        # Nettoyage lors de l'arrêt
        if robot_controller_node is not None:
            robot_controller_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            print("Controller node shutdown complete.")

if __name__ == '__main__':
    # Point d'entrée si le script est exécuté directement
    main()
