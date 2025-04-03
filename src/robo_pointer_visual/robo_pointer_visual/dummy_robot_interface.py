import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Importe le message String

class DummyRobotInterfaceNode(Node):
    """
    Ce nœud simule une interface robot très simple.
    Il s'abonne au topic /robot_command et affiche la commande reçue.
    Dans le futur, il pourrait interagir avec LeRobot ou une simulation.
    """
    def __init__(self):
        super().__init__('dummy_robot_interface')
        self.get_logger().info('Dummy Robot Interface node has started.')

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
        self.get_logger().info(f'>>> Received Command: [{command}] - (Simulating robot action...) <<<')

        # === Logique future ===
        # Ici, on traduirait la commande textuelle ("MOVE_LEFT", "CENTERED"...)
        # en appels concrets à LeRobot ou à un contrôleur de simulation.
        # Par exemple:
        # if command == "MOVE_LEFT":
        #    call_lerobot_move_relative(dx=-0.01)
        # elif command == "CENTERED":
        #    call_lerobot_stop()
        # etc.
        # ====================

def main(args=None):
    rclpy.init(args=args)
    dummy_robot_interface_node = DummyRobotInterfaceNode()
    try:
        rclpy.spin(dummy_robot_interface_node)
    except KeyboardInterrupt:
        pass
    finally:
        dummy_robot_interface_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
