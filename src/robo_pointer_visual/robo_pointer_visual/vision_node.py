# Fichier: ~/ros2_ws/src/robo_pointer_visual/robo_pointer_visual/vision_node.py
# Version: Utilise YOLOv8 pour un suivi d'objet robuste

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point

from ultralytics import YOLO

class VisionNode(Node):
    """
    Ce nœud lit les images d'une webcam (en gérant les déconnexions),
    détecte un objet cible spécifié à l'aide de YOLOv8, et publie :
    - L'image brute sur /image_raw
    - L'image avec les détections dessinées sur /image_debug
    - Les coordonnées (x, y) du centre de l'objet détecté sur /detected_target_point
      (avec x=-1, y=-1 si aucune cible ou caméra non disponible).
    """
    def __init__(self):
        # Initialisation du nœud ROS 2
        super().__init__('vision_node')
        self.get_logger().info('Vision node with YOLOv8 starting, attempting to access webcam...')

        # --- Paramètres de Configuration ---
        self.declare_parameter('camera_index', '/dev/robot_camera') # Défaut: utiliser le lien symbolique
        self.declare_parameter('publish_rate_hz', 20.0) # Fréquence de publication
        self.declare_parameter('yolo_model', 'yolov8n.pt') # Modèle à utiliser (ex: yolov8n.pt pour la version nano)
        self.declare_parameter('target_class_name', 'cup') # Classe d'objet à détecter (ex: 'cup', 'bottle', 'cell phone')
        self.declare_parameter('confidence_threshold', 0.6) # Seuil de confiance minimal pour valider une détection
        
        # Récupérer les valeurs des paramètres
        self.camera_index_param = self.get_parameter('camera_index').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        yolo_model_name = self.get_parameter('yolo_model').get_parameter_value().string_value
        self.target_class_name = self.get_parameter('target_class_name').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        
        self.get_logger().info(f"Using YOLO Model: {yolo_model_name}")
        self.get_logger().info(f"Targeting Class: '{self.target_class_name}' with confidence > {self.confidence_threshold}")

        # --- Initialisation du modèle YOLOv8 ---
        try:
            self.model = YOLO(yolo_model_name)
            self.get_logger().info('YOLOv8 model loaded successfully.')
        except Exception as e:
            self.get_logger().fatal(f'Failed to load YOLO model: {e}. Shutting down.')
            # Une exception ici est critique, on ne peut pas continuer.
            if rclpy.ok():
                rclpy.shutdown()
            return

        # --- Initialisation OpenCV & CvBridge ---
        self.bridge = CvBridge()
        self.camera_capture_source = None # Variable membre pour stocker la source (int ou str)
        self.cap = None # Objet VideoCapture, initialisé à None

        # Déterminer la source de capture à partir du paramètre
        try:
            numeric_index = int(self.camera_index_param)
            self.camera_capture_source = numeric_index
            self.get_logger().info(f"Parameter 'camera_index' is an integer: {self.camera_capture_source}.")
        except ValueError:
            self.camera_capture_source = self.camera_index_param
            self.get_logger().info(f"Parameter 'camera_index' is a string: '{self.camera_capture_source}'.")

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
        if self.cap is not None and self.cap.isOpened():
            self.get_logger().info('Releasing previous capture device...')
            self.cap.release()
            self.cap = None

        try:
            self.get_logger().info(f"Attempting to open capture device: {self.camera_capture_source}")
            self.cap = cv2.VideoCapture(self.camera_capture_source)

            if self.cap is None or not self.cap.isOpened():
                self.get_logger().warn(f"Failed to open webcam: {self.camera_capture_source}. Will retry later.")
                self.cap = None
            else:
                self.get_logger().info(f'Successfully opened webcam: {self.camera_capture_source}')
        except Exception as e:
            self.get_logger().error(f'Exception while trying to open webcam: {e}')
            self.cap = None


    def timer_callback(self):
        """
        Callback appelé périodiquement par le timer.
        Lit une image, utilise YOLOv8 pour détecter la cible, et publie les résultats.
        Tente de rouvrir la caméra si la lecture échoue.
        """
        target_point_msg = Point(x=-1.0, y=-1.0, z=0.0)
        current_stamp = self.get_clock().now().to_msg()
        
        # --- Vérification et lecture de la caméra ---
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().warn("Capture device not open. Attempting to reopen...", throttle_duration_sec=5)
            self.reopen_capture()
            if self.cap is None or not self.cap.isOpened():
                 self.target_publisher.publish(target_point_msg)
                 return

        try:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.get_logger().warn("Failed to grab frame, possible disconnect. Releasing capture object.")
                if self.cap: self.cap.release()
                self.cap = None
                self.target_publisher.publish(target_point_msg)
                return
        except Exception as e_read:
             self.get_logger().error(f"Exception during frame reading: {e_read}")
             if self.cap: self.cap.release()
             self.cap = None
             self.target_publisher.publish(target_point_msg)
             return

        # --- Traitement de l'image ---
        try:
            # Retourner l'image si nécessaire
            frame = cv2.flip(frame, -1)

            # Publication de l'image brute
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            ros_image_msg.header.stamp = current_stamp
            ros_image_msg.header.frame_id = 'camera_frame'
            self.image_publisher.publish(ros_image_msg)

            # --- Détection et suivi avec YOLOv8 ---
            # 'persist=True' permet au tracker de se souvenir des objets entre les appels.
            # 'verbose=False' évite d'imprimer les logs de YOLO dans la console ROS.
            results = self.model.track(frame, persist=True, verbose=False)

            # La méthode .plot() dessine les boîtes, labels et IDs directement sur l'image.
            debug_frame = results[0].plot()

            target_found = False
            highest_confidence = 0.0 # Pour suivre uniquement l'objet le plus probable

            # Vérifier si des objets ont été détectés et s'ils ont un ID de suivi
            if results[0].boxes is not None and results[0].boxes.id is not None:
                boxes = results[0].boxes.xywh.cpu() # Format: [centre_x, centre_y, largeur, hauteur]
                confidences = results[0].boxes.conf.cpu()
                class_ids = results[0].boxes.cls.cpu().tolist()
                
                for box, conf, cls_id in zip(boxes, confidences, class_ids):
                    class_name = self.model.names[int(cls_id)]
                    
                    if class_name == self.target_class_name and conf > self.confidence_threshold:
                        if conf > highest_confidence:
                            highest_confidence = conf
                            target_point_msg.x = float(box[0])
                            target_point_msg.y = float(box[1])
                            target_found = True

            if target_found:
                self.get_logger().debug(f"Target '{self.target_class_name}' found at ({target_point_msg.x:.0f}, {target_point_msg.y:.0f}) with confidence {highest_confidence:.2f}")
            else:
                self.get_logger().debug('No valid target detected in this frame.')
            
            # Publication de l'image de debug
            debug_image_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            debug_image_msg.header.stamp = current_stamp
            debug_image_msg.header.frame_id = 'camera_frame'
            self.debug_image_publisher.publish(debug_image_msg)

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        except Exception as e_proc:
             self.get_logger().error(f'Error during image processing with YOLO: {e_proc}')
             target_point_msg.x = -1.0
             target_point_msg.y = -1.0
        
        # Publication des coordonnées de la cible (trouvée ou par défaut)
        self.target_publisher.publish(target_point_msg)


    def destroy_node(self):
        """Nettoyage à l'arrêt du nœud."""
        self.get_logger().info('Shutting down vision node...')
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('Webcam capture released.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    vision_node = None
    try:
        vision_node = VisionNode()
        if rclpy.ok():
            rclpy.spin(vision_node)
    except KeyboardInterrupt:
         if vision_node: vision_node.get_logger().info('Ctrl+C detected, shutting down.')
         else: print("Ctrl+C detected during node initialization.")
    except Exception as e:
         logger = rclpy.logging.get_logger('vision_node_main')
         logger.fatal(f"Unhandled exception: {e}")
         import traceback
         traceback.print_exc()
    finally:
        if vision_node is not None:
            vision_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Vision node main process finished.")


if __name__ == '__main__':
    main()