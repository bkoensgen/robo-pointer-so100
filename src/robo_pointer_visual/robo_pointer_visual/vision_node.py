# Fichier: ~/ros2_ws/src/robo_pointer_visual/robo_pointer_visual/vision_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Point # Importe le message Point

class VisionNode(Node):
    """
    Ce nœud lit les images d'une webcam, détecte les objets rouges,
    calcule le centre de l'objet rouge le plus grand, et publie :
    - L'image brute sur /image_raw
    - L'image avec les détections dessinées sur /image_debug
    - Les coordonnées (x, y) du centre détecté sur /detected_target_point
      (avec x=-1, y=-1 si aucune cible n'est détectée).
    """
    def __init__(self):
        # Initialisation du nœud ROS 2
        super().__init__('vision_node')
        self.get_logger().info('Vision node has started, attempting to access webcam...')

        # --- Paramètres de Configuration ---
        # Déclare des paramètres ROS 2 pour la configuration (plus flexible)
        self.declare_parameter('camera_index', '0')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('detection_threshold_area', 500)
        # Paramètres pour les seuils HSV (plus facile à ajuster via les paramètres ROS)
        self.declare_parameter('hsv_lower_red1', [0, 120, 70])
        self.declare_parameter('hsv_upper_red1', [10, 255, 255])
        self.declare_parameter('hsv_lower_red2', [170, 120, 70])
        self.declare_parameter('hsv_upper_red2', [180, 255, 255])

        # Récupérer les valeurs des paramètres
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.min_area = self.get_parameter('detection_threshold_area').get_parameter_value().integer_value
        hsv_lower_red1 = self.get_parameter('hsv_lower_red1').get_parameter_value().integer_array_value
        hsv_upper_red1 = self.get_parameter('hsv_upper_red1').get_parameter_value().integer_array_value
        hsv_lower_red2 = self.get_parameter('hsv_lower_red2').get_parameter_value().integer_array_value
        hsv_upper_red2 = self.get_parameter('hsv_upper_red2').get_parameter_value().integer_array_value

        # Convertir les listes de seuils en tableaux NumPy
        self.lower_red1 = np.array(hsv_lower_red1, dtype=np.uint8)
        self.upper_red1 = np.array(hsv_upper_red1, dtype=np.uint8)
        self.lower_red2 = np.array(hsv_lower_red2, dtype=np.uint8)
        self.upper_red2 = np.array(hsv_upper_red2, dtype=np.uint8)

        # --- Initialisation OpenCV & CvBridge ---
        self.bridge = CvBridge()
        camera_capture_source = None
        try:
            # Essayer de convertir la chaîne en entier
            camera_capture_source = int(self.camera_index)
            self.get_logger().info(f"Parameter 'camera_index' is an integer: {camera_capture_source}. Using as camera index.")
        except ValueError:
            # Si la conversion échoue, on suppose que c'est un chemin de périphérique
            camera_capture_source = self.camera_index
            self.get_logger().info(f"Parameter 'camera_index' is a string: '{camera_capture_source}'. Using as device path.")
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                raise IOError(f"Cannot open webcam using source: {camera_capture_source}")
            self.get_logger().info(f'Successfully opened webcam using source: {camera_capture_source}')
        except Exception as e:
            self.get_logger().error(f'Failed to open webcam: {e}')
            raise e

        # --- Publishers ROS ---
        self.image_publisher = self.create_publisher(Image, '/image_raw', 10)
        self.debug_image_publisher = self.create_publisher(Image, '/image_debug', 10)
        self.target_publisher = self.create_publisher(Point, '/detected_target_point', 10)
        self.get_logger().info('Publishing images on /image_raw, /image_debug')
        self.get_logger().info('Publishing target coordinates on /detected_target_point')

        # --- Timer pour la boucle principale ---
        if self.publish_rate <= 0:
             self.get_logger().warn("Publish rate is zero or negative, timer not created.")
             self.timer = None
        else:
             self.timer_period = 1.0 / self.publish_rate
             self.timer = self.create_timer(self.timer_period, self.timer_callback)
             self.get_logger().info(f'Processing images at ~{self.publish_rate:.1f} Hz')

    def timer_callback(self):
        """
        Callback appelé périodiquement par le timer.
        Lit une image, détecte le rouge, calcule le centre, et publie les résultats.
        """
        # Initialiser le message Point avec des valeurs par défaut ("pas de cible")
        target_point_msg = Point()
        target_point_msg.x = -1.0
        target_point_msg.y = -1.0
        target_point_msg.z = 0.0 # Non utilisé pour l'instant

        # Essayer de lire une image
        ret, frame = self.cap.read()

        # Horodatage actuel (même si l'image n'est pas valide, pour le message Point)
        current_stamp = self.get_clock().now().to_msg()

        if ret and frame is not None:
            try:
            
            	# --- 0. Retourner l'image ---
                flip_code = 1
                frame = cv2.flip(frame, flip_code)

                # --- 1. Publication de l'image originale ---
                ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                ros_image_msg.header.stamp = current_stamp
                ros_image_msg.header.frame_id = 'camera_frame' # Repère de la caméra
                self.image_publisher.publish(ros_image_msg)

                # Créer une copie pour les dessins de débogage
                debug_frame = frame.copy()

                # --- 2. Conversion BGR vers HSV ---
                hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                # --- 3. Création des masques pour les plages rouges ---
                mask1 = cv2.inRange(hsv_frame, self.lower_red1, self.upper_red1)
                mask2 = cv2.inRange(hsv_frame, self.lower_red2, self.upper_red2)

                # --- 4. Combinaison des deux masques (OU logique) ---
                red_mask = cv2.bitwise_or(mask1, mask2)

                # 4.1. Définir un noyau (structuring element) - 5x5 est un bon point de départ
                kernel = np.ones((5,5), np.uint8)

                # 4.2. Appliquer l'érosion (enlève le bruit blanc)
                mask_eroded = cv2.erode(red_mask, kernel, iterations = 1)

                # 4.3. Appliquer la dilatation (restaure la taille de l'objet principal)
                mask_processed = cv2.dilate(mask_eroded, kernel, iterations = 1)

                # --- 5. Trouver les contours dans le masque rouge ---
                # Utiliser _ pour ignorer la hiérarchie si on ne s'en sert pas
                contours, _ = cv2.findContours(mask_processed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                # --- 6. Traitement si des contours sont trouvés ---
                if contours:
                    # Trouver le plus grand contour (basé sur l'aire)
                    largest_contour = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(largest_contour)

                    # Ignorer les contours trop petits (bruit)
                    if area > self.min_area:
                        # Calculer le centre (centroïde) du plus grand contour
                        M = cv2.moments(largest_contour)
                        # Vérifier que M["m00"] n'est pas nul pour éviter la division par zéro
                        if M["m00"] > 1e-5: # Utiliser une petite marge pour la robustesse flottante
                            target_center_x = int(M["m10"] / M["m00"])
                            target_center_y = int(M["m01"] / M["m00"])

                            # Mettre à jour le message Point avec les coordonnées trouvées
                            target_point_msg.x = float(target_center_x)
                            target_point_msg.y = float(target_center_y)
                            # target_point_msg.z reste 0.0

                            # Dessiner sur l'image de débogage
                            cv2.drawContours(debug_frame, [largest_contour], -1, (0, 255, 0), 2) # Contour en vert
                            cv2.circle(debug_frame, (target_center_x, target_center_y), 5, (0, 0, 255), -1) # Centre en rouge plein

                            # Log seulement si on trouve une cible valide (optionnel)
                            # self.get_logger().debug(f'Red object detected at ({target_center_x}, {target_center_y}) with area {area:.0f}')

                # --- 7. Publication de l'image de debug (avec ou sans dessins) ---
                debug_image_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
                debug_image_msg.header.stamp = current_stamp # Utiliser le même timestamp
                debug_image_msg.header.frame_id = 'camera_frame'
                self.debug_image_publisher.publish(debug_image_msg)

            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge Error: {e}')
            except Exception as e:
                 self.get_logger().error(f'Error during image processing: {e}')

        else:
            # Si la lecture échoue, on publie quand même l'image de debug (sera vide ou la dernière valide?)
            # et le point cible par défaut. Mieux : ne rien publier pour l'image si ret=False?
            # Pour l'instant, on log juste.
             self.get_logger().warn('Failed to grab frame from webcam')

        # --- 8. Toujours publier le message Point (cible ou défaut) ---
        self.target_publisher.publish(target_point_msg)


    def destroy_node(self):
        """Nettoyage à l'arrêt du nœud."""
        self.get_logger().info('Shutting down vision node...')
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('Webcam released.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    vision_node = None # Initialiser à None
    try:
        vision_node = VisionNode()
        if rclpy.ok(): # Vérifier si l'init a réussi (pas d'erreur webcam)
            rclpy.spin(vision_node)
    except KeyboardInterrupt:
        print("Ctrl+C detected, shutting down.")
    except Exception as e:
         # Log l'erreur qui a pu survenir pendant l'init ou spin
         if vision_node:
              vision_node.get_logger().fatal(f"Unhandled exception: {e}")
         else:
              print(f"Unhandled exception during node creation: {e}")
    finally:
        # Assure le nettoyage même en cas d'erreur
        if vision_node is not None:
             vision_node.destroy_node()
        if rclpy.ok(): # Vérifier si rclpy est toujours initialisé
             rclpy.shutdown()
             print("rclpy shutdown complete.")

if __name__ == '__main__':
    # Ce bloc est exécuté seulement si le script est lancé directement
    main()
