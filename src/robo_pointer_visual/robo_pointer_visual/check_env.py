# ~/ros2_ws/src/robo_pointer_visual/robo_pointer_visual/check_env.py
import rclpy
from rclpy.node import Node
import sys
import os # Pour vérifier la variable d'environnement aussi

class EnvCheckNode(Node):
    def __init__(self):
        super().__init__('env_check_node')
        self.get_logger().info("--- Environment Check ---")
        # Affiche quel exécutable Python est utilisé
        self.get_logger().info(f"Python Executable: {sys.executable}")
        self.get_logger().info("--- sys.path (chemins de recherche des modules) ---")
        # Affiche tous les chemins où Python cherche les modules
        for path in sys.path:
            self.get_logger().info(f"- {path}")
        self.get_logger().info("------")
        # Essaye d'importer tqdm directement ici pour voir
        try:
            import tqdm
            self.get_logger().info("-> Import de 'tqdm' DIRECTEMENT dans ce noeud : RÉUSSI.")
        except ModuleNotFoundError:
            self.get_logger().error("-> Import de 'tqdm' DIRECTEMENT dans ce noeud : ÉCHEC (ModuleNotFoundError).")
        self.get_logger().info("------")
        # Affiche la variable d'environnement PYTHONPATH vue par le noeud
        pythonpath_env = os.environ.get('PYTHONPATH', 'Non définie')
        self.get_logger().info(f"Variable d'environnement PYTHONPATH vue: '{pythonpath_env}'")
        self.get_logger().info("--- Fin du Check ---")

def main(args=None):
    rclpy.init(args=args)
    node = EnvCheckNode()
    # On n'a pas besoin de le faire tourner en boucle, juste afficher les infos
    # rclpy.spin(node)
    # On attend un peu que les logs sortent (pas idéal mais simple)
    import time
    time.sleep(1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
