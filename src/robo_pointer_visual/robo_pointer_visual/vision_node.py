# Fichier: ~/ros2_ws/src/robo_pointer_visual/robo_pointer_visual/vision_node.py
# Version: Ajout de la résilience aux déconnexions/reconnexions de la caméra USB

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Point # Importe le message Point

class VisionNode(Node):
    """
    Ce nœud lit les images d'une webcam (en gérant les déconnexions),
    détecte les objets rouges, calcule le centre du plus grand, et publie :
    - L'image brute sur /image_raw
    - L'image avec les détections dessinées sur /image_debug
    - Les coordonnées (x, y) du centre détecté sur /detected_target_point
      (avec x=-1, y=-1 si aucune cible ou caméra non disponible).
    """
    def __init__(self):
        # Initialisation du nœud ROS 2
        super().__init__('vision_node')
        self.get_logger().info('Vision node starting, attempting to access webcam...')

        # --- Paramètres de Configuration ---
        self.declare_parameter('camera_index', '/dev/robot_camera') # Défaut: utiliser le lien symbolique
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('detection_threshold_area', 500)
        self.declare_parameter('hsv_lower_red1', [0, 120, 70])
        self.declare_parameter('hsv_upper_red1', [10, 255, 255])
        self.declare_parameter('hsv_lower_red2', [170, 120, 70])
        self.declare_parameter('hsv_upper_red2', [180, 255, 255])

        # Récupérer les valeurs des paramètres
        self.camera_index_param = self.get_parameter('camera_index').get_parameter_value().string_value
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
        self.camera_capture_source = None # Variable membre pour stocker la source (int ou str)
        self.cap = None # Objet VideoCapture, initialisé à None

        # Déterminer la source de capture à partir du paramètre
        try:
            # Essayer de convertir la chaîne en entier (pour les index numériques)
            numeric_index = int(self.camera_index_param)
            self.camera_capture_source = numeric_index # Stocker l'entier
            self.get_logger().info(f"Parameter 'camera_index' is an integer: {self.camera_capture_source}. Using as camera index.")
        except ValueError:
            # Si la conversion échoue, on suppose que c'est un chemin de périphérique
            self.camera_capture_source = self.camera_index_param # Stocker la chaîne
            self.get_logger().info(f"Parameter 'camera_index' is a string: '{self.camera_capture_source}'. Using as device path.")

        # Tenter une première ouverture de la caméra
        self.reopen_capture()

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


    def reopen_capture(self):
        """Tente d'ouvrir ou de rouvrir la capture vidéo."""
        # Libérer l'ancienne capture si elle existe et est ouverte
        if self.cap is not None and self.cap.isOpened():
            self.get_logger().info('Releasing previous capture device...')
            self.cap.release()
            self.cap = None # S'assurer qu'il est None avant de réessayer

        try:
            self.get_logger().info(f"Attempting to open capture device: {self.camera_capture_source}")
            # Essayer d'ouvrir avec la source stockée (int ou str)
            self.cap = cv2.VideoCapture(self.camera_capture_source)

            if self.cap is None or not self.cap.isOpened():
                # Si l'ouverture échoue, logger un avertissement et garder self.cap à None
                self.get_logger().warn(f"Failed to open webcam using source: {self.camera_capture_source}. Will retry later.")
                self.cap = None
            else:
                # Si l'ouverture réussit, logger le succès
                self.get_logger().info(f'Successfully opened webcam using source: {self.camera_capture_source}')
                # Configurer des propriétés si nécessaire (ex: résolution, mais peut échouer)
                # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        except Exception as e:
            # Capturer toute exception pendant l'ouverture
            self.get_logger().error(f'Exception while trying to open webcam: {e}')
            self.cap = None # Assurer que cap est None en cas d'exception


    def timer_callback(self):
        """
        Callback appelé périodiquement par le timer.
        Lit une image, détecte le rouge, calcule le centre, et publie les résultats.
        Tente de rouvrir la caméra si la lecture échoue.
        """
        # Initialiser le message Point avec des valeurs par défaut ("pas de cible")
        target_point_msg = Point()
        target_point_msg.x = -1.0
        target_point_msg.y = -1.0
        target_point_msg.z = 0.0 # Non utilisé pour l'instant

        current_stamp = self.get_clock().now().to_msg()
        frame = None
        ret = False

        # --- Vérifier si la caméra est ouverte, sinon tenter de rouvrir ---
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().warn("Capture device not open. Attempting to reopen...", throttle_duration_sec=5)
            self.reopen_capture()
            # Si toujours pas ouverte après la tentative, publier "pas de cible" et sortir
            if self.cap is None or not self.cap.isOpened():
                 self.target_publisher.publish(target_point_msg)
                 return # Attendre le prochain cycle

        # --- Si la caméra est ouverte, essayer de lire une image ---
        try:
            if self.cap is not None and self.cap.isOpened():
                ret, frame = self.cap.read()
                if not ret:
                    # Si la lecture échoue (ret=False), considérer comme une déconnexion
                    self.get_logger().warn("Failed to grab frame, possible disconnect. Releasing capture object.")
                    self.cap.release() # Libérer l'objet potentiellement invalide
                    self.cap = None    # Mettre à None pour forcer reopen au prochain cycle
                    self.target_publisher.publish(target_point_msg) # Publier "pas de cible"
                    return # Sortir pour ce cycle
            else:
                 # Ne devrait pas arriver ici grâce à la vérification précédente, mais sécurité
                 self.get_logger().error("Capture object is None or not opened before reading.")
                 self.target_publisher.publish(target_point_msg)
                 return

        except Exception as e_read:
             # Capturer une exception potentielle pendant la lecture elle-même
             self.get_logger().error(f"Exception during frame reading: {e_read}")
             if self.cap is not None: self.cap.release()
             self.cap = None
             self.target_publisher.publish(target_point_msg)
             return

        # --- Si la lecture a réussi (ret=True et frame is not None) ---
        if ret and frame is not None:
            try:
                # --- 0. Retourner l'image (si nécessaire) ---
                # flip_code = 1 # 1 pour horizontal, 0 pour vertical, -1 pour les deux
                # frame = cv2.flip(frame, flip_code)

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

                # --- 4. Combinaison des deux masques et traitement morphologique ---
                red_mask = cv2.bitwise_or(mask1, mask2)
                kernel = np.ones((5,5), np.uint8) # Noyau 5x5
                mask_eroded = cv2.erode(red_mask, kernel, iterations = 1)
                mask_processed = cv2.dilate(mask_eroded, kernel, iterations = 1)

                # --- 5. Trouver les contours dans le masque traité ---
                contours, _ = cv2.findContours(mask_processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # EXTERNAL est souvent suffisant

                # --- 6. Traitement si des contours sont trouvés ---
                target_found = False
                if contours:
                    largest_contour = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(largest_contour)

                    if area > self.min_area:
                        M = cv2.moments(largest_contour)
                        if M["m00"] > 1e-5: # Éviter division par zéro
                            target_center_x = int(M["m10"] / M["m00"])
                            target_center_y = int(M["m01"] / M["m00"])

                            # Mettre à jour le message Point avec les coordonnées trouvées
                            target_point_msg.x = float(target_center_x)
                            target_point_msg.y = float(target_center_y)
                            target_found = True # Marquer que la cible est trouvée

                            # Dessiner sur l'image de débogage
                            cv2.drawContours(debug_frame, [largest_contour], -1, (0, 255, 0), 2) # Contour en vert
                            cv2.circle(debug_frame, (target_center_x, target_center_y), 5, (0, 0, 255), -1) # Centre en rouge

                            self.get_logger().debug(f'Target found at ({target_center_x}, {target_center_y}), area={area:.0f}')

                if not target_found:
                     self.get_logger().debug('No valid target detected in this frame.')

                # --- 7. Publication de l'image de debug (avec ou sans dessins) ---
                debug_image_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
                debug_image_msg.header.stamp = current_stamp # Utiliser le même timestamp
                debug_image_msg.header.frame_id = 'camera_frame'
                self.debug_image_publisher.publish(debug_image_msg)

            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge Error: {e}')
            except Exception as e_proc:
                 self.get_logger().error(f'Error during image processing: {e_proc}')
                 # Si erreur pendant traitement, on publie quand même "pas de cible"
                 target_point_msg.x = -1.0
                 target_point_msg.y = -1.0

        else:
            # Ce cas ne devrait plus être atteint grâce aux return précédents
            self.get_logger().error('Logical error: Reached end of callback without valid frame.')
            target_point_msg.x = -1.0
            target_point_msg.y = -1.0

        # --- 8. Toujours publier le message Point (cible trouvée ou défaut -1.0) ---
        self.target_publisher.publish(target_point_msg)


    def destroy_node(self):
        """Nettoyage à l'arrêt du nœud."""
        self.get_logger().info('Shutting down vision node...')
        # S'assurer que la caméra est libérée
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('Webcam capture released.')
        super().destroy_node()

# --- Main Entry Point ---
def main(args=None):
    rclpy.init(args=args)
    vision_node = None # Initialiser à None
    try:
        vision_node = VisionNode()
        # Pas besoin de vérifier rclpy.ok() ici, car si init échoue, une exception sera levée
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
         if vision_node: vision_node.get_logger().info('Ctrl+C detected, shutting down.')
         else: print("Ctrl+C detected during node initialization.")
    except Exception as e:
         # Log l'erreur qui a pu survenir pendant l'init ou spin
         logger = rclpy.logging.get_logger('vision_node_main') # Utiliser un logger générique ici
         logger.fatal(f"Unhandled exception: {e}")
         import traceback
         traceback.print_exc()
    finally:
        # Assurer le nettoyage même en cas d'erreur
        if vision_node is not None:
            # Vérifier si le nœud existe et n'est pas déjà détruit
            node_destroyed = True # Assumer détruit par défaut
            try:
                 if vision_node: node_destroyed = vision_node.is_destroyed()
            except Exception as e_check_destroyed:
                 rclpy.logging.get_logger('vision_node_main').warn(f"Could not check if node is destroyed: {e_check_destroyed}")

            if not node_destroyed:
                vision_node.get_logger().info("Executing final cleanup via destroy_node()...")
                try:
                    vision_node.destroy_node()
                    vision_node.get_logger().info("destroy_node() completed.")
                except Exception as e_destroy:
                    vision_node.get_logger().error(f"Exception during destroy_node(): {e_destroy}")
        # Arrêter rclpy s'il est actif
        try:
             if rclpy.ok():
                 rclpy.shutdown()
                 print("rclpy shutdown sequence initiated.")
        except Exception as e_shutdown:
             print(f"Error during final rclpy shutdown: {e_shutdown}")
        print("Vision node main finished.")


if __name__ == '__main__':
    main()