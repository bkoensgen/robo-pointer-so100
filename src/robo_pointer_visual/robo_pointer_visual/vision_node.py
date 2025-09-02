import torch
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from ultralytics import YOLO
import threading
import time
import queue

class VisionNode(Node):
    """
    Nœud de vision par ordinateur.
    - Découple la capture et le traitement de l'image dans un thread dédié pour des performances maximales.
    - Utilise YOLO pour la détection d'objets sur le GPU.
    - Publie les résultats à une fréquence stable définie par un timer ROS.
    """
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info('Vision node starting with async processing.')
        # --- Paramètres ---
        self.declare_parameter('camera_index', '/dev/video0')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('device', 'auto')  # 'auto' | 'cuda' | 'cpu'
        self.declare_parameter('target_class_name', 'bottle')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('flip_code', -1)
        self.declare_parameter('persistence_frames_to_acquire', 3)
        self.declare_parameter('persistence_frames_to_lose', 5)
        self.declare_parameter('camera_backend', 'v4l2')  # 'auto', 'v4l2', 'gstreamer'
        self.declare_parameter('video_fourcc', 'MJPG')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('frame_rate', 30.0)
        
        camera_index_param = self.get_parameter('camera_index').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        yolo_model_name = self.get_parameter('yolo_model').get_parameter_value().string_value
        device_param = self.get_parameter('device').get_parameter_value().string_value
        self.target_class_name = self.get_parameter('target_class_name').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.flip_code = self.get_parameter('flip_code').get_parameter_value().integer_value
        self.frames_to_acquire = self.get_parameter('persistence_frames_to_acquire').get_parameter_value().integer_value
        self.frames_to_lose = self.get_parameter('persistence_frames_to_lose').get_parameter_value().integer_value
        
        # Récupération des nouveaux paramètres
        self.camera_backend = self.get_parameter('camera_backend').get_parameter_value().string_value
        self.video_fourcc = self.get_parameter('video_fourcc').get_parameter_value().string_value
        self.frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value
        
        self.get_logger().info(f"Targeting Class: '{self.target_class_name}' with confidence > {self.confidence_threshold}")
        # Initialisation du modèle YOLO avec sélection du device
        try:
            chosen_device = None
            if device_param.lower() == 'auto':
                chosen_device = 'cuda' if torch.cuda.is_available() else 'cpu'
            elif device_param.lower() in ('cuda', 'cpu'):
                # Respecter le choix explicite mais vérifier CUDA
                if device_param.lower() == 'cuda' and not torch.cuda.is_available():
                    self.get_logger().warn('CUDA not available, falling back to CPU')
                    chosen_device = 'cpu'
                else:
                    chosen_device = device_param.lower()
            else:
                chosen_device = 'cuda' if torch.cuda.is_available() else 'cpu'
                self.get_logger().warn(f"Unknown device '{device_param}', using '{chosen_device}'")

            self.get_logger().info(f"Loading YOLO model '{yolo_model_name}' on device: {chosen_device}")
            self.model = YOLO(yolo_model_name)
            self.model.to(chosen_device)
            if chosen_device == 'cuda':
                try:
                    self.model.half()
                except Exception:
                    # Certaines combinaisons peuvent refuser half-precision: rester en FP32
                    self.get_logger().warn('Half precision not enabled; running in FP32 on CUDA')
            self.class_names = self.model.names
        except Exception as e:
            self.get_logger().fatal(f'Failed to load/configure YOLO model: {e}.'); rclpy.shutdown(); return
        self.bridge = CvBridge()
        try: self.camera_capture_source = int(camera_index_param)
        except ValueError: self.camera_capture_source = camera_index_param
        
        # --- Logique Asynchrone ---
        self.cap = None
        self.latest_frame = None
        self.latest_debug_frame = None
        self.frame_queue = queue.Queue(maxsize=1)
        self.consecutive_failures = 0
        self.max_consecutive_failures = 5
        self.last_known_target_point = Point(x=-1.0, y=-1.0, z=0.0)
        self.data_lock = threading.Lock()
        self.is_running = True
        self.processing_thread = threading.Thread(target=self.processing_worker)
        self.processing_thread.start()
        # Thread dédié à la capture
        self.capture_thread = threading.Thread(target=self.capture_worker)
        self.capture_thread.start()

        # --- Publishers ---
        self.image_publisher = self.create_publisher(Image, '/image_raw', 10)
        self.debug_image_publisher = self.create_publisher(Image, '/image_debug', 10)
        self.target_publisher = self.create_publisher(Point, '/detected_target_point', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_callback)
        self.get_logger().info('YOLOv8 vision node running...')
    
    def _configure_camera(self):
        """Configure la caméra avec les paramètres spécifiés"""
        # Déterminer le backend OpenCV
        backend_flag = 0  # Par défaut (auto)
        if self.camera_backend == 'v4l2':
            backend_flag = cv2.CAP_V4L2
        elif self.camera_backend == 'gstreamer':
            backend_flag = cv2.CAP_GSTREAMER
        
        self.get_logger().info(f"Attempting to open camera with backend: {self.camera_backend}")
        self.cap = cv2.VideoCapture(self.camera_capture_source, backend_flag)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().warn(f"Failed to open camera with {self.camera_backend} backend, trying default")
            self.cap = cv2.VideoCapture(self.camera_capture_source)
        
        if not self.cap.isOpened():
            self.get_logger().fatal("Failed to open camera after all attempts")
            return False
        
        # Configurer le FOURCC
        fourcc = cv2.VideoWriter_fourcc(*self.video_fourcc)
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        
        # Configurer résolution et FPS
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        # Vérifier et logger la configuration réelle
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        actual_fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join([chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)])
        
        self.get_logger().info(
            f"Camera configured: {actual_width}x{actual_height}@{actual_fps}fps, "
            f"FOURCC: {fourcc_str} (requested: {self.video_fourcc})"
        )
        
        # Attendre que la caméra se stabilise
        time.sleep(1.0)
        return True

    def capture_worker(self):
        """Capture en continu et alimente la queue sans jamais bloquer."""
        if not self._configure_camera():
            rclpy.shutdown(); return
        while self.is_running:
            ret, frame = self.cap.read()
            if not ret: continue
            if self.frame_queue.full():
                try: self.frame_queue.get_nowait()   # drop l’ancienne
                except queue.Empty: pass
            self.frame_queue.put(frame)

    def processing_worker(self):
        """Ce thread tourne en boucle pour traiter les images aussi vite que possible."""
        self.get_logger().info("Processing worker thread started.")
        self.get_logger().info(f"Worker thread is opening camera at: {self.camera_capture_source}")
        
        target_is_acquired = False
        detection_counter = 0
        while self.is_running:
            try:
                frame = self.frame_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            
            results = self.model(frame, verbose=False, conf=self.confidence_threshold)[0]
            debug_frame = frame.copy()
            best_target_center = None
            largest_area = 0
            detection_this_frame = False
            for box in results.boxes:
                class_id = int(box.cls[0])
                if self.class_names[class_id] == self.target_class_name:
                    detection_this_frame = True
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    area = (x2 - x1) * (y2 - y1)
                    if area > largest_area:
                        largest_area = area
                        best_target_center = ((x1 + x2) // 2, (y1 + y2) // 2)
            if detection_this_frame:
                detection_counter = min(detection_counter + 1, self.frames_to_acquire)
            else:
                detection_counter = max(detection_counter - 1, -self.frames_to_lose)
            if detection_counter >= self.frames_to_acquire:
                target_is_acquired = True
            elif detection_counter <= -self.frames_to_lose:
                target_is_acquired = False
            with self.data_lock:
                self.latest_frame = frame
                for box in results.boxes:
                    x1,y1,x2,y2 = map(int, box.xyxy[0])
                    cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                self.latest_debug_frame = debug_frame
                if target_is_acquired and best_target_center:
                    self.last_known_target_point = Point(x=float(best_target_center[0]), y=float(best_target_center[1]), z=0.0)
                else:
                    self.last_known_target_point = Point(x=-1.0, y=-1.0, z=0.0)
        self.get_logger().info("Worker thread loop finished. Releasing camera now.")
        self.cap.release()
        self.get_logger().info("Camera released by worker thread.")
    
    def publish_callback(self):
        """Ce callback est appelé à la fréquence de publication et ne fait que publier."""
        with self.data_lock:
            raw_frame = self.latest_frame
            debug_frame = self.latest_debug_frame
            target_point = self.last_known_target_point
        if raw_frame is not None:
            if self.flip_code != 99:
                raw_frame_to_publish = cv2.flip(raw_frame, self.flip_code)
                debug_frame_to_publish = cv2.flip(debug_frame, self.flip_code)
            else:
                raw_frame_to_publish = raw_frame
                debug_frame_to_publish = debug_frame
            self.image_publisher.publish(self.bridge.cv2_to_imgmsg(raw_frame_to_publish, "bgr8"))
            self.debug_image_publisher.publish(self.bridge.cv2_to_imgmsg(debug_frame_to_publish, "bgr8"))
        self.target_publisher.publish(target_point)
    
    def destroy_node(self):
        self.get_logger().info('Shutting down vision node...')
        self.is_running = False
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join()
        if hasattr(self, 'processing_thread'):
            self.processing_thread.join()
        if self.cap and self.cap.isOpened():
            self.get_logger().warn("Releasing camera from destroy_node as a fallback.")
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    vision_node = None
    executor = SingleThreadedExecutor()
    try:
        vision_node = VisionNode()
        executor.add_node(vision_node)
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        if executor: executor.shutdown()
        if vision_node: vision_node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__ == '__main__':
    main()
