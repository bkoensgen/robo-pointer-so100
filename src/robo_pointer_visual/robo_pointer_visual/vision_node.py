import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from ultralytics import YOLO

class VisionNode(Node):
    """
    Nœud de vision par ordinateur.
    - Capture le flux vidéo d'une caméra.
    - Utilise un modèle YOLO pour détecter des objets.
    - Applique un filtre de persistance pour stabiliser la détection.
    - Publie l'image brute, une image de débogage et les coordonnées du centre de la cible.
    """
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info('Vision node starting.')

        # --- Paramètres ---
        self.declare_parameter('camera_index', '/dev/camera_robot')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('yolo_model', 'yolov8l.pt')
        self.declare_parameter('target_class_name', 'bottle')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('flip_code', -1)
        self.declare_parameter('persistence_frames_to_acquire', 3)
        self.declare_parameter('persistence_frames_to_lose', 5)

        # Récupération des paramètres
        camera_index_param = self.get_parameter('camera_index').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        yolo_model_name = self.get_parameter('yolo_model').get_parameter_value().string_value
        self.target_class_name = self.get_parameter('target_class_name').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.flip_code = self.get_parameter('flip_code').get_parameter_value().integer_value
        self.frames_to_acquire = self.get_parameter('persistence_frames_to_acquire').get_parameter_value().integer_value
        self.frames_to_lose = self.get_parameter('persistence_frames_to_lose').get_parameter_value().integer_value
        
        self.get_logger().info(f"Targeting Class: '{self.target_class_name}' with confidence > {self.confidence_threshold}")

        # Initialisations
        try:
            self.model = YOLO(yolo_model_name)
            self.class_names = self.model.names
        except Exception as e:
            self.get_logger().fatal(f'Failed to load YOLO model: {e}.'); rclpy.shutdown(); return
        
        self.bridge = CvBridge()
        self.cap = None
        try: self.camera_capture_source = int(camera_index_param)
        except ValueError: self.camera_capture_source = camera_index_param
        self.reopen_capture()
        
        self.target_is_acquired = False
        self.detection_counter = 0
        self.last_known_target_point = Point(x=-1.0, y=-1.0, z=0.0)

        self.image_publisher = self.create_publisher(Image, '/image_raw', 10)
        self.debug_image_publisher = self.create_publisher(Image, '/image_debug', 10)
        self.target_publisher = self.create_publisher(Point, '/detected_target_point', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate if self.publish_rate > 0 else 0.05, self.timer_callback)
        self.get_logger().info('YOLOv8 vision node running...')

    def reopen_capture(self):
        if self.cap and self.cap.isOpened(): self.cap.release()
        
        self.cap = cv2.VideoCapture(self.camera_capture_source, cv2.CAP_V4L2)
        
        if self.cap and self.cap.isOpened():
            self.get_logger().info(f'Webcam {self.camera_capture_source} opened. Forcing optimal parameters...')
            
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not (self.cap and self.cap.isOpened()):
            self.get_logger().warn(f"Failed to open webcam: {self.camera_capture_source}", throttle_duration_sec=5)
            self.cap = None
        else:
            # On vérifie les paramètres qui ont réellement été appliqués
            width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            self.get_logger().info(f'Successfully opened webcam with parameters: {int(width)}x{int(height)} @ {fps:.2f} FPS')

    def timer_callback(self):
        """Callback principal: récupère une image, la traite et publie les résultats."""
        if self.cap is None:
            self.reopen_capture()
            if self.cap is None:
                self.update_persistence(False)
                self.target_publisher.publish(self.last_known_target_point)
                return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("Failed to grab frame.", throttle_duration_sec=5)
            if self.cap: self.cap.release()
            self.cap = None
            self.update_persistence(False)
            self.target_publisher.publish(self.last_known_target_point)
            return

        # Exécuter la logique de traitement et de publication
        self.process_and_publish_frame(frame)

    def process_and_publish_frame(self, frame):
        """Traite une image, met à jour l'état et publie les topics nécessaires."""
        # 1. Exécuter l'inférence YOLO sur l'image brute
        results = self.model(frame, verbose=False, conf=self.confidence_threshold)

        # 2. Préparer une image de débogage et y dessiner les résultats
        debug_frame = frame.copy()
        best_target_center, detection_this_frame = self.draw_and_find_target(debug_frame, results)
        
        # 3. Mettre à jour la persistance de la cible
        self.update_persistence(detection_this_frame)
        if detection_this_frame:
            self.last_known_target_point.x = float(best_target_center[0])
            self.last_known_target_point.y = float(best_target_center[1])

        # 4. Gérer le 'flip' des images juste avant la publication
        if self.flip_code != 99:
            debug_frame = cv2.flip(debug_frame, self.flip_code)
        
        raw_frame_to_publish = frame.copy()
        if self.flip_code != 99:
            raw_frame_to_publish = cv2.flip(raw_frame_to_publish, self.flip_code)
        
        # 5. Publier tous les messages
        self.image_publisher.publish(self.bridge.cv2_to_imgmsg(raw_frame_to_publish, "bgr8"))
        self.debug_image_publisher.publish(self.bridge.cv2_to_imgmsg(debug_frame, "bgr8"))

        if not self.target_is_acquired:
            self.target_publisher.publish(Point(x=-1.0, y=-1.0, z=0.0))
        else:
            self.target_publisher.publish(self.last_known_target_point)

    def draw_and_find_target(self, debug_frame, results):
        """
        Dessine les boîtes de détection sur l'image de débogage et trouve
        la meilleure cible (la plus grande de la classe voulue).
        """
        best_target_center = None
        largest_area = 0
        detection_this_frame = False

        status_text = f"Status: Searching ({self.detection_counter})"
        status_color = (0, 255, 255) # Jaune
        if self.target_is_acquired:
            status_text = "Status: ACQUIRED"
            status_color = (0, 255, 0) # Vert
        cv2.putText(debug_frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)
            
        for box in results[0].boxes:
            class_id = int(box.cls[0])
            class_name = self.class_names[class_id]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            
            box_color = (255, 100, 100)
            if class_name == self.target_class_name:
                detection_this_frame = True
                box_color = (0, 255, 255)
                if self.target_is_acquired:
                    box_color = (0, 255, 0)
                
                area = (x2 - x1) * (y2 - y1)
                if area > largest_area:
                    largest_area = area
                    best_target_center = ((x1 + x2) // 2, (y1 + y2) // 2)

            cv2.rectangle(debug_frame, (x1, y1), (x2, y2), box_color, 2)
            label = f"{class_name} {box.conf[0]:.2f}"
            cv2.putText(debug_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
        if self.target_is_acquired and best_target_center:
             cv2.circle(debug_frame, best_target_center, 7, (0, 0, 255), -1)

        return best_target_center, detection_this_frame
        
    def update_persistence(self, detection_this_frame):
        """
        Met à jour un compteur pour filtrer les détections parasites.
        La cible est "acquise" après plusieurs détections successives et
        "perdue" après plusieurs images sans détection.
        """
        if detection_this_frame:
            self.detection_counter = min(self.detection_counter + 1, self.frames_to_acquire)
        else:
            self.detection_counter = max(self.detection_counter - 1, -self.frames_to_lose)
            
        if self.detection_counter >= self.frames_to_acquire:
            if not self.target_is_acquired:
                self.get_logger().info(f"Target ACQUIRED (seen for {self.detection_counter} frames)")
            self.target_is_acquired = True
        elif self.detection_counter <= -self.frames_to_lose:
            if self.target_is_acquired:
                self.get_logger().info(f"Target LOST (unseen for {abs(self.detection_counter)} frames)")
            self.target_is_acquired = False
            
    def destroy_node(self):
        self.get_logger().info('Shutting down vision node...')
        if self.cap: self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    vision_node = None
    try:
        vision_node = VisionNode()
        rclpy.spin(vision_node)
    except KeyboardInterrupt: pass
    finally:
        if vision_node: vision_node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
