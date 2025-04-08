# Fichier: ~/ros2_ws/src/robo_pointer_visual/robo_pointer_visual/real_robot_interface.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Pour recevoir la commande

from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus, FeetechMotorsBusConfig # Hypothèse du chemin

# TODO (Benjamin): Vérifier/Ajuster le VRAI port série du bras Leader
LEADER_ARM_PORT = "/dev/ttyACM0" # Exemple, à vérifier !

# Définition des moteurs du bras Leader (copié de la config LeRobot)
LEADER_ARM_MOTORS = {
    "shoulder_pan": [1, "sts3215"],
    "shoulder_lift": [2, "sts3215"],
    "elbow_flex": [3, "sts3215"],
    "wrist_flex": [4, "sts3215"],
    "wrist_roll": [5, "sts3215"],
    "gripper": [6, "sts3215"],
}

class RealRobotInterfaceNode(Node):
    """
    Ce nœud fait l'interface avec le bras robotique réel (Leader seul).
    Il s'abonne à /robot_command, traduit la commande en appel moteur
    via FeetechMotorsBus, et envoie la commande au matériel.
    """
    def __init__(self):
        super().__init__('real_robot_interface')
        self.get_logger().info('Real Robot Interface node starting...')

        self.leader_bus = None # Initialiser à None

        leader_config = FeetechMotorsBusConfig(port=LEADER_ARM_PORT, motors=LEADER_ARM_MOTORS)
        
        self.leader_bus = FeetechMotorsBus(leader_config)

        try:
            self.leader_bus.connect()
            self.get_logger().info(f"Successfully connected to leader arm bus on {LEADER_ARM_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to leader arm bus on {LEADER_ARM_PORT}: {e}")
            # Que faire ici ? Lever l'erreur ? Juste continuer sans bus fonctionnel ?
            # Pour l'instant, on logge juste l'erreur. Le callback vérifiera si le bus est connecté.
            self.leader_bus = None # Assurer que leader_bus est None si connexion échoue    

        # --- Subscriber à la commande ---
        self.command_subscription = self.create_subscription(
            String,
            '/robot_command',
            self.command_callback,
            10
        )
        self.get_logger().info('Subscribed to /robot_command')

    def command_callback(self, msg):
        """Callback appelé à chaque message String reçu sur /robot_command."""
        command = msg.data
        self.get_logger().info(f'Received Command: [{command}]')

        # Vérifier si le bus est connecté avant d'envoyer une commande
        if self.leader_bus is None or not self.leader_bus.is_connected:
             self.get_logger().warn("Leader bus not connected, cannot send command.")
             return


        if command == "MOVE_LEFT":
            try:
                target_angle = 10.0 # Degrés ? Ticks ? À vérifier !
                motor_name_to_move = "shoulder_pan" # ID 1
                self.get_logger().info(f"Attempting to write Goal_Position={target_angle} to {motor_name_to_move}")
                # Supposons que write prend des degrés pour l'instant
                self.leader_bus.write("Goal_Position", target_angle, motor_name_to_move)
                self.get_logger().info(f"Command sent successfully to {motor_name_to_move}")
            except Exception as e:
                self.get_logger().error(f"Failed to send command to {motor_name_to_move}: {e}")
        elif command == "MOVE_RIGHT":
            try:
                target_angle = -10.0 # Degrés ? Ticks ? À vérifier !
                motor_name_to_move = "shoulder_pan" # ID 1
                self.get_logger().info(f"Attempting to write Goal_Position={target_angle} to {motor_name_to_move}")
                # Supposons que write prend des degrés pour l'instant
                self.leader_bus.write("Goal_Position", target_angle, motor_name_to_move)
                self.get_logger().info(f"Command sent successfully to {motor_name_to_move}")
            except Exception as e:
                self.get_logger().error(f"Failed to send command to {motor_name_to_move}: {e}")

        elif command == "CENTERED" or command == "IDLE":
            self.leader_bus.write("Torque_Enable", 0, None) # Pour tous les moteurs)

    def destroy_node(self):
        """Nettoyage à l'arrêt du nœud."""
        self.get_logger().info('Shutting down Real Robot Interface node...')
        if self.leader_bus is not None and self.leader_bus.is_connected:
            try:
                self.leader_bus.disconnect()
                self.get_logger().info("Leader bus disconnected.")
            except Exception as e:
                self.get_logger().error(f"Error disconnecting leader bus: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    real_robot_interface_node = None
    try:
        real_robot_interface_node = RealRobotInterfaceNode()
        # Vérifier si le nœud a pu s'initialiser (connexion bus) avant de spinner
        # (Une meilleure gestion serait de lever une exception dans __init__ si la connexion échoue)
        if real_robot_interface_node.leader_bus is not None and real_robot_interface_node.leader_bus.is_connected:
             rclpy.spin(real_robot_interface_node)
        else:
             real_robot_interface_node.get_logger().error("Initialization failed (bus not connected). Shutting down.")

    except KeyboardInterrupt:
        if real_robot_interface_node:
             real_robot_interface_node.get_logger().info('Ctrl+C detected, shutting down interface node.')
        else:
             print('Ctrl+C detected during node initialization.')
    except Exception as e:
        if real_robot_interface_node:
            real_robot_interface_node.get_logger().error(f'Interface node error: {e}')
        else:
            print(f'Error creating interface node: {e}')
    finally:
        if real_robot_interface_node is not None:
            real_robot_interface_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            print("Interface node shutdown complete.")

if __name__ == '__main__':
    main()
