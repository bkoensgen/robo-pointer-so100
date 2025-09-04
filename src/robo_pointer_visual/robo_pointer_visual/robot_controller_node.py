import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rcl_interfaces.msg import (
    ParameterDescriptor,
    FloatingPointRange,
    IntegerRange,
    SetParametersResult,
)
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np
import math
import time
import traceback

from .kinematics import (
    calculate_fk_wrist, calculate_ik, 
    calculate_gravity_compensation_adj_deg, 
    calculate_wrist_angle_for_horizontal,
    calculate_aim_offset
)

class RobotControllerNode(Node):
    """
    Nœud "Cerveau" du robot.
    - Reçoit les coordonnées d'une cible détectée par le nœud de vision.
    - Utilise la cinématique inverse pour calculer les angles des articulations nécessaires.
    - Applique des compensations (gravité, position de la caméra).
    - Publie les angles cibles vers l'interface matérielle du robot.
    """
    def __init__(self):
        super().__init__('robot_controller_node')
        self.get_logger().info('Robot Controller node has started.')

        # --- Paramètres de Contrôle ---
        # Descripteurs numériques pour validation
        gt0 = ParameterDescriptor(
            description='Strictly positive scalar',
            floating_point_range=[FloatingPointRange(from_value=1e-9, to_value=1e9, step=0.0)],
        )
        deg_any = ParameterDescriptor(
            description='Angle in degrees (-360..360)',
            floating_point_range=[FloatingPointRange(from_value=-360.0, to_value=360.0, step=0.0)],
        )
        offset_nonneg = ParameterDescriptor(
            description='Non-negative offset (m)',
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0, step=0.0)],
        )
        img_dim = ParameterDescriptor(
            description='Positive image dimension (px)',
            integer_range=[IntegerRange(from_value=16, to_value=16384, step=1)],
        )

        # Topics (relatifs par défaut pour supporter le namespacing)
        self.declare_parameter('target_topic', 'detected_target_point', ParameterDescriptor(description='Input target point topic'))
        self.declare_parameter('joint_states_topic', 'joint_states', ParameterDescriptor(description='Input joint states topic'))
        self.declare_parameter('target_joint_angles_topic', 'target_joint_angles', ParameterDescriptor(description='Output target joint angles topic'))

        # Paramètres numériques avec bornes
        self.declare_parameter('pixel_to_cartesian_scale_x', 0.0001, gt0)
        self.declare_parameter('pixel_to_cartesian_scale_y', 0.0001, gt0)
        self.declare_parameter('camera_tilt_angle_deg', 30.0, deg_any)
        self.declare_parameter('camera_forward_offset_m', 0.05, offset_nonneg)
        self.declare_parameter('k_gravity_id2', 12.0, gt0)
        self.declare_parameter('k_gravity_id3', 6.0, gt0)
        self.declare_parameter('lift_angle_min_deg', 0.0, deg_any)
        self.declare_parameter('lift_angle_max_deg', 130.0, deg_any)
        self.declare_parameter('elbow_angle_min_deg', 0.0, deg_any)
        self.declare_parameter('elbow_angle_max_deg', 110.0, deg_any)
        self.declare_parameter('pan_angle_min_deg', -110.0, deg_any)
        self.declare_parameter('pan_angle_max_deg', 110.0, deg_any)
        self.declare_parameter('wrist_angle_min_deg', -100.0, deg_any)
        self.declare_parameter('wrist_angle_max_deg', 100.0, deg_any)
        self.declare_parameter('image_width', 640, img_dim)
        self.declare_parameter('image_height', 480, img_dim)

        # Optional PID-based visual servoing (kept off by default)
        self.declare_parameter('use_pid_control', False)
        # Pan PID
        self.declare_parameter('kp_pan', 0.006)
        self.declare_parameter('ki_pan', 0.0)
        self.declare_parameter('kd_pan', 0.001)
        self.declare_parameter('i_pan_min', -5.0)
        self.declare_parameter('i_pan_max', 5.0)
        # Vertical PID (acts on pixel Y; converted to meters via scale_y and to IK target)
        self.declare_parameter('kp_vert', 0.004)
        self.declare_parameter('ki_vert', 0.0)
        self.declare_parameter('kd_vert', 0.0015)
        self.declare_parameter('i_vert_min', -5.0)
        self.declare_parameter('i_vert_max', 5.0)
        # Per-joint speed limits (deg/s) using dotted parameter names for YAML mapping compatibility
        self.declare_parameter('joint_speed_max_deg_s.pan', 60.0)
        self.declare_parameter('joint_speed_max_deg_s.lift', 45.0)
        self.declare_parameter('joint_speed_max_deg_s.elbow', 45.0)
        self.declare_parameter('joint_speed_max_deg_s.wrist', 90.0)

        # Safety and smoothing
        self.declare_parameter('dead_zone_px', 8)
        self.declare_parameter('no_detection_behavior', 'hold')  # hold|home
        self.declare_parameter('no_detection_timeout_s', 3.0)

        # Récupération des paramètres
        # Topics
        self.target_topic = self.get_parameter('target_topic').get_parameter_value().string_value
        self.joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        self.target_joint_angles_topic = self.get_parameter('target_joint_angles_topic').get_parameter_value().string_value

        self.scale_x = self.get_parameter('pixel_to_cartesian_scale_x').get_parameter_value().double_value
        self.scale_y = self.get_parameter('pixel_to_cartesian_scale_y').get_parameter_value().double_value
        self.camera_tilt_deg = self.get_parameter('camera_tilt_angle_deg').get_parameter_value().double_value
        self.camera_offset_m = self.get_parameter('camera_forward_offset_m').get_parameter_value().double_value
        self.k_gravity_id2 = self.get_parameter('k_gravity_id2').get_parameter_value().double_value
        self.k_gravity_id3 = self.get_parameter('k_gravity_id3').get_parameter_value().double_value
        self.lift_min_deg = self.get_parameter('lift_angle_min_deg').get_parameter_value().double_value
        self.lift_max_deg = self.get_parameter('lift_angle_max_deg').get_parameter_value().double_value
        self.elbow_min_deg = self.get_parameter('elbow_angle_min_deg').get_parameter_value().double_value
        self.elbow_max_deg = self.get_parameter('elbow_angle_max_deg').get_parameter_value().double_value
        self.pan_min_deg = self.get_parameter('pan_angle_min_deg').get_parameter_value().double_value
        self.pan_max_deg = self.get_parameter('pan_angle_max_deg').get_parameter_value().double_value
        self.wrist_min_deg = self.get_parameter('wrist_angle_min_deg').get_parameter_value().double_value
        self.wrist_max_deg = self.get_parameter('wrist_angle_max_deg').get_parameter_value().double_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value

        # PID parameters
        self.use_pid = bool(self.get_parameter('use_pid_control').get_parameter_value().bool_value)
        self.kp_pan = float(self.get_parameter('kp_pan').get_parameter_value().double_value)
        self.ki_pan = float(self.get_parameter('ki_pan').get_parameter_value().double_value)
        self.kd_pan = float(self.get_parameter('kd_pan').get_parameter_value().double_value)
        self.i_pan_min = float(self.get_parameter('i_pan_min').get_parameter_value().double_value)
        self.i_pan_max = float(self.get_parameter('i_pan_max').get_parameter_value().double_value)
        self.kp_vert = float(self.get_parameter('kp_vert').get_parameter_value().double_value)
        self.ki_vert = float(self.get_parameter('ki_vert').get_parameter_value().double_value)
        self.kd_vert = float(self.get_parameter('kd_vert').get_parameter_value().double_value)
        self.i_vert_min = float(self.get_parameter('i_vert_min').get_parameter_value().double_value)
        self.i_vert_max = float(self.get_parameter('i_vert_max').get_parameter_value().double_value)
        self.max_speed_pan = float(self.get_parameter('joint_speed_max_deg_s.pan').get_parameter_value().double_value)
        self.max_speed_lift = float(self.get_parameter('joint_speed_max_deg_s.lift').get_parameter_value().double_value)
        self.max_speed_elbow = float(self.get_parameter('joint_speed_max_deg_s.elbow').get_parameter_value().double_value)
        self.max_speed_wrist = float(self.get_parameter('joint_speed_max_deg_s.wrist').get_parameter_value().double_value)

        self.dead_zone_px = int(self.get_parameter('dead_zone_px').get_parameter_value().integer_value)
        self.no_detection_behavior = str(self.get_parameter('no_detection_behavior').get_parameter_value().string_value).lower()
        self.no_detection_timeout_s = float(self.get_parameter('no_detection_timeout_s').get_parameter_value().double_value)

        # Internal PID state
        self._ex_prev = 0.0
        self._ey_prev = 0.0
        self._ix = 0.0
        self._iy = 0.0
        self._last_time_ns = self.get_clock().now().nanoseconds
        self._last_home_tick_ns = self._last_time_ns
        self._last_detection_time_ns = self._last_time_ns
        self._homing_active = False
        
        self.current_joint_states = None
        self.pan_motor_name = "shoulder_pan"
        self.lift_motor_name = "shoulder_lift"
        self.elbow_motor_name = "elbow_flex"
        self.wrist_motor_name = "wrist_flex"
        
        initial_lift, initial_elbow = 107.0, 85.0
        initial_wrist = calculate_wrist_angle_for_horizontal(initial_lift, initial_elbow)
        self.initial_pose_deg = {
            self.pan_motor_name: 0.0, self.lift_motor_name: initial_lift,
            self.elbow_motor_name: initial_elbow, self.wrist_motor_name: initial_wrist
        }
        self.target_subscription = self.create_subscription(Point, self.target_topic, self.target_callback, 10)
        self.joint_state_subscription = self.create_subscription(JointState, self.joint_states_topic, self.joint_state_callback, 10)
        self.target_angles_publisher = self.create_publisher(JointState, self.target_joint_angles_topic, 10)
        
        # Timer pour envoyer la position de départ de manière robuste
        self.initial_pose_timer = self.create_timer(0.5, self.send_initial_pose_when_ready)
        # Timer de sécurité: homing si la détection est absente trop longtemps
        self.homing_timer = self.create_timer(0.1, self.homing_tick)

        self.get_logger().info(
            f"Ready. PID={'on' if self.use_pid else 'off'}; max_speeds(deg/s)="
            f" pan={self.max_speed_pan}, lift={self.max_speed_lift},"
            f" elbow={self.max_speed_elbow}, wrist={self.max_speed_wrist}"
        )

        # Validation dynamique (cohérence min/max et bornes supplémentaires)
        self.add_on_set_parameters_callback(self._on_set_parameters)

    def send_initial_pose_when_ready(self):
        """
        Vérifie si un abonné (le robot) est prêt avant d'envoyer la position
        de départ. S'annule après le premier envoi réussi.
        """
        if self.target_angles_publisher.get_subscription_count() > 0:
            # Attente supplémentaire pour la caméra
            self.get_logger().info("Waiting 2 seconds for camera to stabilize before moving...")
            time.sleep(2.0)
            self.get_logger().info(f"Subscriber ready. Sending initial pose: { {k: round(v, 1) for k, v in self.initial_pose_deg.items()} }")
            
            target_msg = JointState()
            target_msg.header.stamp = self.get_clock().now().to_msg()
            target_msg.name = list(self.initial_pose_deg.keys())
            target_msg.position = [math.radians(angle) for angle in self.initial_pose_deg.values()]
            
            self.target_angles_publisher.publish(target_msg)
            
            self.initial_pose_timer.cancel()
            self.get_logger().info("Initial pose sent. Controller is now active.")
        else:
            self.get_logger().warn("Waiting for real_robot_interface to subscribe...", throttle_duration_sec=2)

    def joint_state_callback(self, msg: JointState):
        """Met à jour l'état actuel des articulations du robot."""
        self.current_joint_states = msg

    def _on_set_parameters(self, params: list[Parameter]) -> SetParametersResult:
        """Valide les mises à jour de paramètres (bornes et cohérence)."""
        def as_float(p: Parameter) -> float:
            return float(p.value)
        def as_int(p: Parameter) -> int:
            return int(p.value)
        for p in params:
            n = p.name
            if n in ('pixel_to_cartesian_scale_x', 'pixel_to_cartesian_scale_y', 'k_gravity_id2', 'k_gravity_id3'):
                if p.type_ != Parameter.Type.DOUBLE or as_float(p) <= 0.0:
                    return SetParametersResult(successful=False, reason=f'{n} must be > 0')
            elif n == 'camera_forward_offset_m':
                if p.type_ != Parameter.Type.DOUBLE or as_float(p) < 0.0:
                    return SetParametersResult(successful=False, reason=f'{n} must be >= 0')
            elif n in ('camera_tilt_angle_deg','lift_angle_min_deg','lift_angle_max_deg','elbow_angle_min_deg','elbow_angle_max_deg','pan_angle_min_deg','pan_angle_max_deg','wrist_angle_min_deg','wrist_angle_max_deg'):
                if p.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(successful=False, reason=f'{n} must be a float degree value')
                v = as_float(p)
                if not (-360.0 <= v <= 360.0):
                    return SetParametersResult(successful=False, reason=f'{n} must be within [-360, 360]')
            elif n in ('image_width','image_height'):
                if p.type_ != Parameter.Type.INTEGER or as_int(p) <= 0:
                    return SetParametersResult(successful=False, reason=f'{n} must be a positive integer')
            elif n in ('kp_pan','ki_pan','kd_pan','kp_vert','ki_vert','kd_vert','i_pan_min','i_pan_max','i_vert_min','i_vert_max'):
                if p.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(successful=False, reason=f'{n} must be a float')
            elif n in ('use_pid_control',):
                if p.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(successful=False, reason=f'{n} must be a bool')
            elif n in (
                'joint_speed_max_deg_s.pan',
                'joint_speed_max_deg_s.lift',
                'joint_speed_max_deg_s.elbow',
                'joint_speed_max_deg_s.wrist',
            ):
                if p.type_ != Parameter.Type.DOUBLE or as_float(p) <= 0.0:
                    return SetParametersResult(successful=False, reason=f'{n} must be > 0 (deg/s)')
            elif n == 'dead_zone_px':
                if p.type_ != Parameter.Type.INTEGER or as_int(p) < 0:
                    return SetParametersResult(successful=False, reason='dead_zone_px must be a non-negative integer')
            elif n == 'no_detection_timeout_s':
                if p.type_ != Parameter.Type.DOUBLE or as_float(p) <= 0.0:
                    return SetParametersResult(successful=False, reason='no_detection_timeout_s must be > 0')
            elif n == 'no_detection_behavior':
                if p.type_ != Parameter.Type.STRING or str(p.value).lower() not in {'hold','home'}:
                    return SetParametersResult(successful=False, reason="no_detection_behavior must be 'hold' or 'home'")

        # Cohérence min <= max (utilise les valeurs courantes)
        lift_min = float(self.get_parameter('lift_angle_min_deg').value)
        lift_max = float(self.get_parameter('lift_angle_max_deg').value)
        elbow_min = float(self.get_parameter('elbow_angle_min_deg').value)
        elbow_max = float(self.get_parameter('elbow_angle_max_deg').value)
        pan_min = float(self.get_parameter('pan_angle_min_deg').value)
        pan_max = float(self.get_parameter('pan_angle_max_deg').value)
        wrist_min = float(self.get_parameter('wrist_angle_min_deg').value)
        wrist_max = float(self.get_parameter('wrist_angle_max_deg').value)
        # Integral clamps
        i_pan_min = float(self.get_parameter('i_pan_min').value)
        i_pan_max = float(self.get_parameter('i_pan_max').value)
        i_vert_min = float(self.get_parameter('i_vert_min').value)
        i_vert_max = float(self.get_parameter('i_vert_max').value)

        if lift_min > lift_max:
            return SetParametersResult(successful=False, reason='lift_angle_min_deg must be <= lift_angle_max_deg')
        if elbow_min > elbow_max:
            return SetParametersResult(successful=False, reason='elbow_angle_min_deg must be <= elbow_angle_max_deg')
        if pan_min > pan_max:
            return SetParametersResult(successful=False, reason='pan_angle_min_deg must be <= pan_angle_max_deg')
        if wrist_min > wrist_max:
            return SetParametersResult(successful=False, reason='wrist_angle_min_deg must be <= wrist_angle_max_deg')
        if i_pan_min > i_pan_max:
            return SetParametersResult(successful=False, reason='i_pan_min must be <= i_pan_max')
        if i_vert_min > i_vert_max:
            return SetParametersResult(successful=False, reason='i_vert_min must be <= i_vert_max')

        # Apply updates to internal fields for dynamic tuning
        for p in params:
            n = p.name
            try:
                if n == 'use_pid_control':
                    self.use_pid = bool(p.value)
                elif n in ('kp_pan','ki_pan','kd_pan'):
                    setattr(self, n, float(p.value))
                elif n in ('kp_vert','ki_vert','kd_vert'):
                    setattr(self, n, float(p.value))
                elif n in ('i_pan_min','i_pan_max','i_vert_min','i_vert_max'):
                    setattr(self, n, float(p.value))
                elif n == 'joint_speed_max_deg_s.pan':
                    self.max_speed_pan = float(p.value)
                elif n == 'joint_speed_max_deg_s.lift':
                    self.max_speed_lift = float(p.value)
                elif n == 'joint_speed_max_deg_s.elbow':
                    self.max_speed_elbow = float(p.value)
                elif n == 'joint_speed_max_deg_s.wrist':
                    self.max_speed_wrist = float(p.value)
            except Exception:
                pass

        # Apply updates to internal fields for dynamic tuning
        for p in params:
            n = p.name
            try:
                if n == 'dead_zone_px':
                    self.dead_zone_px = int(p.value)
                elif n == 'no_detection_timeout_s':
                    self.no_detection_timeout_s = float(p.value)
                elif n == 'no_detection_behavior':
                    self.no_detection_behavior = str(p.value).lower()
            except Exception:
                pass

        return SetParametersResult(successful=True)

    def target_callback(self, msg: Point):
        """Callback principal pour le traitement d'une nouvelle cible."""
        if not self.initial_pose_timer.is_canceled():
            return
        # Handle detection presence
        if msg.x == -1.0:
            # no detection in this frame; leave last_detection_time untouched
            return
        else:
            self._last_detection_time_ns = self.get_clock().now().nanoseconds
            # Leaving homing mode if it was active
            if self._homing_active:
                self._homing_active = False
        if self.current_joint_states is None: return

        try:
            joint_map = {name: math.degrees(pos) for name, pos in zip(self.current_joint_states.name, self.current_joint_states.position)}
            current_pan_deg = joint_map.get(self.pan_motor_name, self.initial_pose_deg[self.pan_motor_name])
            current_lift_deg = joint_map.get(self.lift_motor_name, self.initial_pose_deg[self.lift_motor_name])
            current_elbow_deg = joint_map.get(self.elbow_motor_name, self.initial_pose_deg[self.elbow_motor_name])

            current_xw, current_yw = calculate_fk_wrist(current_lift_deg, current_elbow_deg)

            # Pixel errors relative to image center + dead-zone
            error_x = msg.x - (self.image_width / 2.0)
            error_y = msg.y - (self.image_height / 2.0)
            if abs(error_x) < self.dead_zone_px:
                error_x = 0.0
            if abs(error_y) < self.dead_zone_px:
                error_y = 0.0

            # Time delta for PID and slew-rate limiting
            now_ns = self.get_clock().now().nanoseconds
            dt = max((now_ns - self._last_time_ns) * 1e-9, 1e-3)
            self._last_time_ns = now_ns

            if self.use_pid:
                # --- PAN (horizontal) PID ---
                dex = (error_x - self._ex_prev) / dt
                self._ix = float(np.clip(self._ix + error_x * dt, self.i_pan_min, self.i_pan_max))
                # Sign convention: positive error_x means target to the right -> rotate pan negative
                u_pan_deg_per_s = - (self.kp_pan * error_x + self.kd_pan * dex + self.ki_pan * self._ix)
                max_delta_pan = self.max_speed_pan * dt

                # --- VERTICAL PID (uses IK) ---
                dey = (error_y - self._ey_prev) / dt
                self._iy = float(np.clip(self._iy + error_y * dt, self.i_vert_min, self.i_vert_max))
                # Convert vertical control into a small cartesian delta (meters)
                delta_yw = (self.kp_vert * error_y + self.kd_vert * dey + self.ki_vert * self._iy) * self.scale_y * dt
                perceived_target_yw = current_yw + delta_yw
                perceived_target_xw = current_xw

                # Update previous errors
                self._ex_prev = error_x
                self._ey_prev = error_y
            else:
                # Legacy proportional behavior
                u_pan_deg_per_s = - (error_x * self.scale_x) / max(dt, 1e-3)
                max_delta_pan = self.max_speed_pan * dt
                # L'axe Y de l'image est inversé par rapport à l'axe Y cartésien du robot
                delta_yw = error_y * self.scale_y
                perceived_target_yw = current_yw + delta_yw
                perceived_target_xw = current_xw

            # Compenser le décalage de la caméra pour trouver la cible IK réelle
            final_arm_orientation_deg = -(current_lift_deg + current_elbow_deg)
            offset_x, offset_y = calculate_aim_offset(final_arm_orientation_deg, self.camera_tilt_deg, self.camera_offset_m)
            ik_target_xw = perceived_target_xw - offset_x
            ik_target_yw = perceived_target_yw - offset_y
            
            ik_lift_rad, ik_elbow_rad, ik_ok = calculate_ik(ik_target_xw, ik_target_yw)
            if not ik_ok: return

            ideal_lift_deg = math.degrees(ik_lift_rad)
            ideal_elbow_deg = math.degrees(ik_elbow_rad)

            # Ajouter la compensation de gravité
            gravity_lift_adj, gravity_elbow_adj = calculate_gravity_compensation_adj_deg(
                ideal_lift_deg, ideal_elbow_deg, self.k_gravity_id2, self.k_gravity_id3
            )
            compensated_lift_deg = ideal_lift_deg + gravity_lift_adj
            compensated_elbow_deg = ideal_elbow_deg + gravity_elbow_adj

            # Pan desired (deg) from rate command with slew limit
            pan_desired_deg = current_pan_deg + float(np.clip(u_pan_deg_per_s * dt, -max_delta_pan, max_delta_pan))
            target_wrist_deg = calculate_wrist_angle_for_horizontal(compensated_lift_deg, compensated_elbow_deg)

            # Clip to physical limits first
            pan_phys = float(np.clip(pan_desired_deg, self.pan_min_deg, self.pan_max_deg))
            lift_phys = float(np.clip(compensated_lift_deg, self.lift_min_deg, self.lift_max_deg))
            elbow_phys = float(np.clip(compensated_elbow_deg, self.elbow_min_deg, self.elbow_max_deg))
            wrist_phys = float(np.clip(target_wrist_deg, self.wrist_min_deg, self.wrist_max_deg))

            # Rate-limit per joint (deg/s) relative to current position
            current_wrist_deg = joint_map.get(self.wrist_motor_name, self.initial_pose_deg[self.wrist_motor_name])
            final_pan_deg = current_pan_deg + float(np.clip(pan_phys - current_pan_deg, -self.max_speed_pan * dt, self.max_speed_pan * dt))
            final_lift_deg = current_lift_deg + float(np.clip(lift_phys - current_lift_deg, -self.max_speed_lift * dt, self.max_speed_lift * dt))
            final_elbow_deg = current_elbow_deg + float(np.clip(elbow_phys - current_elbow_deg, -self.max_speed_elbow * dt, self.max_speed_elbow * dt))
            final_wrist_deg = current_wrist_deg + float(np.clip(wrist_phys - current_wrist_deg, -self.max_speed_wrist * dt, self.max_speed_wrist * dt))
            
            self.get_logger().debug(
                f"Target Angles (deg): pan={final_pan_deg:.1f}, lift={final_lift_deg:.1f},"
                f" elbow={final_elbow_deg:.1f}, wrist={final_wrist_deg:.1f}"
            )
            
            target_joint_state_msg = JointState()
            target_joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            target_joint_state_msg.name = [self.pan_motor_name, self.lift_motor_name, self.elbow_motor_name, self.wrist_motor_name]
            target_joint_state_msg.position = [
                math.radians(final_pan_deg), math.radians(final_lift_deg),
                math.radians(final_elbow_deg), math.radians(final_wrist_deg)
            ]
            
            self.target_angles_publisher.publish(target_joint_state_msg)

        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")
            traceback.print_exc()

    def homing_tick(self):
        """Si aucune détection pendant no_detection_timeout_s et mode 'home',
        ramener progressivement le bras à la pose initiale en respectant les vitesses max.
        """
        # Attendre l'envoi initial
        if not self.initial_pose_timer.is_canceled():
            return
        if self.no_detection_behavior != 'home':
            return
        now_ns = self.get_clock().now().nanoseconds
        since_det_s = (now_ns - self._last_detection_time_ns) * 1e-9
        if since_det_s < self.no_detection_timeout_s:
            return

        # Activer homing
        self._homing_active = True
        dt = max((now_ns - self._last_home_tick_ns) * 1e-9, 1e-3)
        self._last_home_tick_ns = now_ns

        # Lire l'état courant; fallback sur initial si inconnu
        joint_map = {}
        if self.current_joint_states is not None:
            joint_map = {name: math.degrees(pos) for name, pos in zip(self.current_joint_states.name, self.current_joint_states.position)}
        current_pan_deg = joint_map.get(self.pan_motor_name, self.initial_pose_deg[self.pan_motor_name])
        current_lift_deg = joint_map.get(self.lift_motor_name, self.initial_pose_deg[self.lift_motor_name])
        current_elbow_deg = joint_map.get(self.elbow_motor_name, self.initial_pose_deg[self.elbow_motor_name])
        current_wrist_deg = joint_map.get(self.wrist_motor_name, self.initial_pose_deg[self.wrist_motor_name])

        # Cible: pose initiale
        tgt_pan = self.initial_pose_deg[self.pan_motor_name]
        tgt_lift = self.initial_pose_deg[self.lift_motor_name]
        tgt_elbow = self.initial_pose_deg[self.elbow_motor_name]
        tgt_wrist = self.initial_pose_deg[self.wrist_motor_name]

        # Progression limitée par vitesses max
        next_pan = current_pan_deg + float(np.clip(tgt_pan - current_pan_deg, -self.max_speed_pan * dt, self.max_speed_pan * dt))
        next_lift = current_lift_deg + float(np.clip(tgt_lift - current_lift_deg, -self.max_speed_lift * dt, self.max_speed_lift * dt))
        next_elbow = current_elbow_deg + float(np.clip(tgt_elbow - current_elbow_deg, -self.max_speed_elbow * dt, self.max_speed_elbow * dt))
        next_wrist = current_wrist_deg + float(np.clip(tgt_wrist - current_wrist_deg, -self.max_speed_wrist * dt, self.max_speed_wrist * dt))

        # Publier la commande
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.pan_motor_name, self.lift_motor_name, self.elbow_motor_name, self.wrist_motor_name]
        msg.position = [
            math.radians(next_pan), math.radians(next_lift), math.radians(next_elbow), math.radians(next_wrist)
        ]
        self.target_angles_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = RobotControllerNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        if executor: executor.shutdown()
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
