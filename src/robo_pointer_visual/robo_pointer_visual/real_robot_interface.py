# Fichier: ~/ros2_ws/src/robo_pointer_visual/robo_pointer_visual/real_robot_interface.py
# Version: Intégration FK/IK avec SDK bas niveau et calibration manuelle (JSON patché)

import numpy as np
import math
import time
import json # Pour lire le fichier de calibration
from pathlib import Path # Pour gérer le chemin du fichier
import enum # Pour CalibrationMode
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

# Utilisation directe de FeetechMotorsBus (pour config et connexion bas niveau)
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus, FeetechMotorsBusConfig
# Importation directe du SDK bas niveau
try:
    import scservo_sdk as scs
except ImportError:
    print("ERREUR: Le paquet scservo_sdk n'est pas installé.")
    print("Veuillez l'installer (il est normalement une dépendance de lerobot).")
    scs = None # Pour éviter les erreurs plus loin si l'import échoue
    # On pourrait lever une exception ici pour arrêter proprement si scs est requis

# Import local de la cinématique
from .kinematics import calculate_fk_wrist, calculate_ik

# --- Constantes et Définitions ---
LEADER_ARM_PORT = "/dev/ttyACM0" # CORRIGÉ: Port série direct
LEADER_ARM_MOTORS = {
    "shoulder_pan": [1, "sts3215"],
    "shoulder_lift": [2, "sts3215"],
    "elbow_flex": [3, "sts3215"],
    "wrist_flex": [4, "sts3215"],
    "wrist_roll": [5, "sts3215"],
    "gripper": [6, "sts3215"],
}
MOTOR_NAMES_ORDER = list(LEADER_ARM_MOTORS.keys())
MODEL_RESOLUTION = {"sts3215": 4096}
HALF_TURN_DEGREE = 180.0
SCS_CONTROL_TABLE = { # Adresses importantes du control table Feetech
    "Torque_Enable": (40, 1),
    "Goal_Position": (42, 2),
    "Present_Position": (56, 2),
}

class CalibrationMode(enum.Enum):
    DEGREE = 0
    LINEAR = 1

# --- Fonctions de Calibration ---

# VERSION CORRIGÉE AVEC GESTION WRAP-AROUND (SANS PRINTS DEBUG INTERNES)
def apply_calibration(raw_values: np.ndarray, motor_names: list[str], calibration_data: dict, motors_def: dict):
    """Convertit les steps bruts lus (int32) en degrés/pourcentage calibrés (float32), gérant le wrap-around."""
    if calibration_data is None:
        print("[ERROR] apply_calibration: calibration_data is None!") # Utiliser print car hors classe Node
        return np.full(len(motor_names), np.nan, dtype=np.float32)

    calibrated_values = np.full(len(motor_names), np.nan, dtype=np.float32)

    for i, name in enumerate(motor_names):
        if name not in calibration_data["motor_names"]:
            print(f"[WARN apply_calib] Motor '{name}' not found in calibration data.")
            continue

        try:
            calib_idx = calibration_data["motor_names"].index(name)
            # Vérifications de base de la cohérence des listes de calibration
            if calib_idx >= len(calibration_data.get("drive_mode", [])) or \
               calib_idx >= len(calibration_data.get("homing_offset", [])) or \
               calib_idx >= len(calibration_data.get("calib_mode", [])):
                print(f"[ERROR apply_calib] Calibration data lists mismatch for motor '{name}'.")
                calibrated_values[i] = np.nan
                continue

            calib_mode_str = calibration_data["calib_mode"][calib_idx]
            try: calib_mode = CalibrationMode[calib_mode_str]
            except KeyError:
                print(f"[ERROR apply_calib] Invalid calib_mode '{calib_mode_str}' for motor '{name}'.")
                calibrated_values[i] = np.nan
                continue

            drive_mode = calibration_data["drive_mode"][calib_idx]
            # Utiliser float() pour plus de robustesse
            raw_zero = float(-calibration_data["homing_offset"][calib_idx])
            _, model = motors_def[name]
            resolution = float(MODEL_RESOLUTION[model])
            current_raw_value = float(raw_values[i])
            steps_per_180_deg = resolution / 2.0

            if calib_mode == CalibrationMode.DEGREE:
                if abs(steps_per_180_deg) < 1e-9:
                     print(f"[ERROR apply_calib] Invalid resolution for motor '{name}'.")
                     calibrated_values[i] = np.nan
                     continue

                delta_raw = current_raw_value - raw_zero
                delta_normalized = delta_raw
                # Gestion du wrap-around
                if delta_raw > steps_per_180_deg:
                    delta_normalized = delta_raw - resolution
                elif delta_raw < -steps_per_180_deg:
                    delta_normalized = delta_raw + resolution

                delta_steps_calibrated = delta_normalized
                # Application du drive_mode
                if drive_mode == 1:
                    delta_steps_calibrated *= -1.0

                calibrated_degrees = delta_steps_calibrated / steps_per_180_deg * HALF_TURN_DEGREE
                calibrated_values[i] = calibrated_degrees

            elif calib_mode == CalibrationMode.LINEAR:
                 if calib_idx >= len(calibration_data.get("start_pos", [])) or \
                    calib_idx >= len(calibration_data.get("end_pos", [])):
                      print(f"[ERROR apply_calib] Missing start/end_pos for linear motor '{name}'.")
                      calibrated_values[i] = np.nan
                      continue
                 start_pos = float(calibration_data["start_pos"][calib_idx])
                 end_pos = float(calibration_data["end_pos"][calib_idx])
                 if abs(end_pos - start_pos) < 1e-6:
                      calibrated_values[i] = 0.0 if current_raw_value == start_pos else 50.0 # Arbitraire
                 else:
                      percentage = (current_raw_value - start_pos) / (end_pos - start_pos) * 100.0
                      calibrated_values[i] = np.clip(percentage, -10.0, 110.0) # Permettre légère marge

        except KeyError as e:
            print(f"[ERROR apply_calib] Missing key in calibration data for '{name}': {e}")
            calibrated_values[i] = np.nan
        except Exception as e:
            print(f"[ERROR apply_calib] Unexpected error processing motor '{name}': {e}")
            import traceback
            traceback.print_exc()
            calibrated_values[i] = np.nan

    return calibrated_values

# VERSION STANDARD (INCHANGÉE)
def revert_calibration(target_values_deg: np.ndarray, motor_names: list[str], calibration_data: dict, motors_def: dict):
    """Convertit les degrés/pourcentage cibles (float32) en steps bruts (int32) pour l'envoi."""
    if calibration_data is None:
         print("[ERROR revert_calib] calibration_data is None!")
         # Retourner None ou un array de zéros/NaN ? None est plus explicite
         return None

    target_steps = np.zeros(len(motor_names), dtype=np.int32)

    for i, name in enumerate(motor_names):
        # Utiliser un step par défaut (ex: milieu de course) si erreur
        default_step = MODEL_RESOLUTION.get(motors_def.get(name, ["", "sts3215"])[1], 4096) // 2

        if name not in calibration_data["motor_names"]:
            print(f"[WARN revert_calib] Motor '{name}' not found in calibration data.")
            target_steps[i] = default_step
            continue

        try:
            calib_idx = calibration_data["motor_names"].index(name)
            # Vérifications de cohérence
            if calib_idx >= len(calibration_data.get("drive_mode", [])) or \
               calib_idx >= len(calibration_data.get("homing_offset", [])) or \
               calib_idx >= len(calibration_data.get("calib_mode", [])):
                print(f"[ERROR revert_calib] Calibration data lists mismatch for motor '{name}'.")
                target_steps[i] = default_step
                continue

            calib_mode_str = calibration_data["calib_mode"][calib_idx]
            try: calib_mode = CalibrationMode[calib_mode_str]
            except KeyError:
                print(f"[ERROR revert_calib] Invalid calib_mode '{calib_mode_str}' for motor '{name}'.")
                target_steps[i] = default_step
                continue

            drive_mode = calibration_data["drive_mode"][calib_idx]
            homing_offset = float(calibration_data["homing_offset"][calib_idx]) # Use float
            _, model = motors_def[name]
            resolution = float(MODEL_RESOLUTION[model]) # Use float
            current_target_deg = float(target_values_deg[i]) # Use float

            if calib_mode == CalibrationMode.DEGREE:
                steps_per_180_deg = resolution / 2.0
                if abs(steps_per_180_deg) < 1e-9:
                     print(f"[ERROR revert_calib] Invalid resolution for motor '{name}'.")
                     target_steps[i] = default_step
                     continue

                step_centered = current_target_deg / HALF_TURN_DEGREE * steps_per_180_deg
                if drive_mode == 1:
                    step_centered *= -1.0
                target_step_raw = step_centered - homing_offset
                # Clipper aux limites physiques du moteur (0 à resolution-1) ?
                # target_step_clipped = np.clip(target_step_raw, 0, resolution - 1)
                # target_steps[i] = int(round(target_step_clipped))
                # Pour l'instant, pas de clipping ici, on suppose que les angles cibles sont déjà limités
                target_steps[i] = int(round(target_step_raw))

            elif calib_mode == CalibrationMode.LINEAR:
                 if calib_idx >= len(calibration_data.get("start_pos", [])) or \
                    calib_idx >= len(calibration_data.get("end_pos", [])):
                      print(f"[ERROR revert_calib] Missing start/end_pos for linear motor '{name}'.")
                      target_steps[i] = default_step
                      continue
                 start_pos = float(calibration_data["start_pos"][calib_idx])
                 end_pos = float(calibration_data["end_pos"][calib_idx])

                 if abs(end_pos - start_pos) < 1e-6:
                     target_steps[i] = int(round(start_pos))
                 else:
                     # Clipper le % cible entre 0 et 100 avant calcul
                     target_percentage_clipped = np.clip(current_target_deg, 0.0, 100.0)
                     target_step_raw = (target_percentage_clipped / 100.0) * (end_pos - start_pos) + start_pos
                     # Clipper aux limites physiques ?
                     # target_step_clipped = np.clip(target_step_raw, 0, resolution - 1)
                     # target_steps[i] = int(round(target_step_clipped))
                     target_steps[i] = int(round(target_step_raw))

        except KeyError as e:
            print(f"[ERROR revert_calib] Missing key in calibration data for '{name}': {e}")
            target_steps[i] = default_step
        except Exception as e:
            print(f"[ERROR revert_calib] Unexpected error processing motor '{name}': {e}")
            import traceback
            traceback.print_exc()
            target_steps[i] = default_step

    return target_steps

# --- Classe du Nœud ROS ---
class RealRobotInterfaceNode(Node):
    """
    Interface avec le bras robotique Feetech réel via SDK bas niveau.
    Utilise FK/IK pour le contrôle vertical coordonné (Lift/Elbow).
    Lit la calibration depuis un fichier JSON (potentiellement patché).
    """
    def __init__(self):
        super().__init__('real_robot_interface')
        self.get_logger().info('Real Robot Interface node starting... (Manual Calibration & Low-Level SDK Mode)')
        self.leader_bus_instance = None # Garder l'instance pour maintenir le port ouvert
        self.port_handler = None
        self.packet_handler = None
        self.calibration_data = None

        if scs is None:
             self.get_logger().fatal("scservo_sdk not found. Cannot initialize node.")
             # Lever une exception ici est plus propre pour arrêter le lancement
             raise ImportError("scservo_sdk is required but not installed.")

        try:
            # 1. Charger la calibration
            # CORRIGÉ : chemin exact + suppression du ".cache" initial si lerobot est directement dans home
            calibration_file_path = Path.home() / "lerobot/.cache/calibration/so100/main_follower.json"
            # Vérification de l'existence avant lecture
            if not calibration_file_path.exists():
                self.get_logger().error(f"Calibration file not found at {calibration_file_path}. Cannot proceed.")
                raise FileNotFoundError(f"Calibration file not found: {calibration_file_path}")

            with open(calibration_file_path, 'r') as f:
                self.calibration_data = json.load(f)
            self.get_logger().info(f"Calibration data loaded from {calibration_file_path}")

            # Vérifications de base du fichier chargé
            expected_keys = ["motor_names", "homing_offset", "drive_mode", "calib_mode"]
            if not all(k in self.calibration_data for k in expected_keys):
                 self.get_logger().error("Calibration file missing required keys!")
                 raise ValueError("Invalid calibration file structure.")
            if self.calibration_data.get("motor_names") != MOTOR_NAMES_ORDER:
                 self.get_logger().error("Motor order mismatch in calibration file!")
                 raise ValueError("Motor order mismatch.")
            # Vérification drive_mode (important!)
            expected_drive_modes = [0, 1, 0, 0, 1, 0] # Pan, Lift, Elbow, WristFlex, WristRoll, Gripper
            current_drive_modes = list(self.calibration_data.get("drive_mode", []))
            if current_drive_modes != expected_drive_modes:
                 self.get_logger().error(f"!!! ERREUR JSON: drive_mode attendu={expected_drive_modes}, trouvé={current_drive_modes}")
                 raise ValueError("Incorrect drive_mode in calibration file.")
            else:
                self.get_logger().info(f"Drive modes confirmés: {current_drive_modes}")


            # 2. Initialiser le bus moteur ET LE GARDER pour maintenir le port ouvert
            # Utiliser la constante corrigée pour le port
            leader_config = FeetechMotorsBusConfig(port=LEADER_ARM_PORT, motors=LEADER_ARM_MOTORS)
            self.leader_bus_instance = FeetechMotorsBus(leader_config)
            self.get_logger().info(f"Connecting to bus on {LEADER_ARM_PORT} to get handlers...")
            self.leader_bus_instance.connect() # Ouvre le port et configure baudrate
            self.port_handler = self.leader_bus_instance.port_handler
            self.packet_handler = self.leader_bus_instance.packet_handler
            # Vérifier que les handlers sont valides
            if self.port_handler is None or self.packet_handler is None:
                 raise RuntimeError("Failed to get valid port/packet handlers from FeetechMotorsBus instance.")
            self.get_logger().info(f"Successfully connected and obtained handlers.")

            # 3. Stabilisation Initiale du Poignet et Activation Torque (via SDK bas niveau)
            try:
                # Moteurs à initialiser (potentiellement tous ceux utilisés)
                motors_to_init = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]
                INITIAL_WRIST_POS_DEG = 0.0 # Poignet droit (pour wrist_flex)
                self.get_logger().info("Initializing motor torque and wrist position...")

                addr_torque = SCS_CONTROL_TABLE["Torque_Enable"][0]
                addr_goal_pos = SCS_CONTROL_TABLE["Goal_Position"][0]

                for name in motors_to_init:
                    motor_id = LEADER_ARM_MOTORS[name][0]

                    # Activer le torque (important de le faire pour tous)
                    # Utiliser TxRx même si on n'attend pas de réponse, c'est plus robuste parfois
                    scs_comm_result, scs_error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, addr_torque, 1)
                    if scs_comm_result != scs.COMM_SUCCESS:
                         self.get_logger().warn(f"Torque enable comm failed for {name}({motor_id}). Res:{scs_comm_result}")
                    elif scs_error != 0:
                         self.get_logger().warn(f"Torque enable motor error for {name}({motor_id}). Err:{scs_error}")
                    else:
                         self.get_logger().debug(f"Torque enabled for {name}({motor_id}).")
                    time.sleep(0.05) # Petite pause après activation torque

                    # Commande initiale *seulement* pour le poignet pour éviter mouvements brusques
                    if name == "wrist_flex":
                        target_steps_wrist_array = revert_calibration(
                            np.array([INITIAL_WRIST_POS_DEG], dtype=np.float32),
                            [name], self.calibration_data, LEADER_ARM_MOTORS
                        )
                        if target_steps_wrist_array is None:
                             self.get_logger().error("Revert calib failed for initial wrist pos.")
                             continue # Ne pas envoyer si erreur

                        target_steps_wrist = target_steps_wrist_array.item()

                        scs_comm_result, scs_error = self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, addr_goal_pos, target_steps_wrist)
                        if scs_comm_result != scs.COMM_SUCCESS:
                             self.get_logger().warn(f"Initial wrist pos comm failed. Res:{scs_comm_result}")
                        elif scs_error != 0:
                            self.get_logger().warn(f"Initial wrist pos motor error. Err:{scs_error}")
                        else:
                             self.get_logger().info(f"Initial position command sent for {name} (Target steps: {target_steps_wrist}).")
                        time.sleep(0.5) # Pause plus longue pour que le poignet atteigne la position

                self.get_logger().info("Motor init complete.")

            except Exception as e_setup:
                # Log plus détaillé en cas d'erreur ici
                self.get_logger().error(f"Error during initial motor setup: {e_setup}")
                import traceback; traceback.print_exc()
                # Peut-être faut-il arrêter si l'init échoue ? Pour l'instant on continue.

        except (FileNotFoundError, ValueError, RuntimeError, ImportError) as e:
            # Capturer les erreurs spécifiques d'init
            self.get_logger().fatal(f"CRITICAL INIT ERROR: {e}")
            import traceback; traceback.print_exc()
            # Essayer de nettoyer ce qui a pu être créé
            if self.leader_bus_instance and getattr(self.leader_bus_instance, 'is_connected', False):
                self.leader_bus_instance.disconnect()
            self.port_handler = None; self.packet_handler = None
            # Relancer l'exception pour que le 'main' sache que l'init a échoué
            raise RuntimeError("Node initialization failed") from e
        except Exception as e:
             # Capturer toute autre erreur inattendue pendant l'init
             self.get_logger().fatal(f"Unexpected error during node initialization: {e}")
             import traceback; traceback.print_exc()
             if self.leader_bus_instance and getattr(self.leader_bus_instance, 'is_connected', False):
                 self.leader_bus_instance.disconnect()
             self.port_handler = None; self.packet_handler = None
             raise RuntimeError("Node initialization failed") from e


        # --- Paramètres ROS 2 ---
        # Déclarer les paramètres utilisés dans le callback ou pour config générale
        self.declare_parameter('pan_increment_scale', 0.1)
        self.declare_parameter('pan_angle_min', -110.0)
        self.declare_parameter('pan_angle_max', 110.0)

        self.declare_parameter('yw_increment_scale', 0.0001) # Gain YW (vertical) - Peut nécessiter ajustement
        self.declare_parameter('lift_angle_min', -90.0) # Limite articulaire physique/sécurité
        self.declare_parameter('lift_angle_max', 90.0)
        self.declare_parameter('elbow_angle_min', 0.0)  # Limite articulaire physique/sécurité
        self.declare_parameter('elbow_angle_max', 150.0)

        self.declare_parameter('target_yw_min', -0.02) # Limite de l'espace de travail cartésien Y
        self.declare_parameter('target_yw_max', 0.20)
        self.declare_parameter('stop_threshold', 0.01) # Seuil pour ignorer petits signaux

        # Lire les paramètres
        self.pan_increment_scale = self.get_parameter('pan_increment_scale').get_parameter_value().double_value
        self.pan_min = self.get_parameter('pan_angle_min').get_parameter_value().double_value
        self.pan_max = self.get_parameter('pan_angle_max').get_parameter_value().double_value

        self.yw_increment_scale = self.get_parameter('yw_increment_scale').get_parameter_value().double_value
        self.lift_min = self.get_parameter('lift_angle_min').get_parameter_value().double_value
        self.lift_max = self.get_parameter('lift_angle_max').get_parameter_value().double_value
        self.elbow_min = self.get_parameter('elbow_angle_min').get_parameter_value().double_value
        self.elbow_max = self.get_parameter('elbow_angle_max').get_parameter_value().double_value

        self.Y_MIN_LIMIT = self.get_parameter('target_yw_min').get_parameter_value().double_value
        self.Y_MAX_LIMIT = self.get_parameter('target_yw_max').get_parameter_value().double_value
        self.stop_threshold = self.get_parameter('stop_threshold').get_parameter_value().double_value

        # Log des paramètres chargés
        self.get_logger().info(f"Pan control: Scale={self.pan_increment_scale:.4f}, Limits=[{self.pan_min:.1f}, {self.pan_max:.1f}] deg")
        self.get_logger().info(f"Lift/Elbow control: YW_Scale={self.yw_increment_scale:.5f}, YW_Limits=[{self.Y_MIN_LIMIT:.3f}, {self.Y_MAX_LIMIT:.3f}] m")
        self.get_logger().info(f"Joint Limits (Clipping): Lift=[{self.lift_min:.1f}, {self.lift_max:.1f}] deg, Elbow=[{self.elbow_min:.1f}, {self.elbow_max:.1f}] deg")
        self.get_logger().info(f"Stop Threshold: {self.stop_threshold:.3f}")

        # --- Subscriber ---
        self.control_signal_subscription = self.create_subscription(
            Vector3,
            '/robot_control_signal', # Topic écouté
            self.control_signal_callback, # Fonction à appeler
            10 # QoS profile (depth)
        )
        self.get_logger().info('Subscribed to /robot_control_signal')
        # PAS de timer ici (supprimé par rapport au Test 1.3)


    def control_signal_callback(self, msg: Vector3):
        """
        Traite un message de contrôle, lit l'état actuel (calibré), calcule FK/IK,
        et commande les moteurs via SDK bas niveau.
        """
        # --- Vérifications Initiales ---
        if self.port_handler is None or self.packet_handler is None or self.calibration_data is None:
            self.get_logger().warn("Handlers or calibration data missing, skip command.", throttle_duration_sec=5)
            return
        if scs is None: # Vérifier si l'import a échoué
             self.get_logger().error("scservo_sdk not loaded, cannot execute callback.", throttle_duration_sec=5)
             return

        signal_pan = msg.x
        signal_lift = msg.y # Signal Y du contrôleur (erreur image)

        # --- 1. Lire l'état actuel (steps bruts) via SDK ---
        try:
            motors_to_read = ["shoulder_pan", "shoulder_lift", "elbow_flex"]
            raw_positions_list = []
            addr_present_pos = SCS_CONTROL_TABLE["Present_Position"][0]
            read_successful = True
            partial_read = False # Flag si au moins une lecture échoue mais d'autres réussissent

            for name in motors_to_read:
                motor_id = LEADER_ARM_MOTORS[name][0]
                value, result, error = self.packet_handler.read2ByteTxRx(self.port_handler, motor_id, addr_present_pos)
                # Note: On pourrait utiliser readSync ici pour lire tous les moteurs en une fois,
                # mais la lecture individuelle est parfois plus facile à déboguer au début.

                if result != scs.COMM_SUCCESS:
                    self.get_logger().warn(f"Read comm failed {name}({motor_id}). Res:{result}", throttle_duration_sec=1.0)
                    raw_positions_list.append(None) # Marquer comme échec
                    read_successful = False
                    partial_read = True
                elif error != 0:
                    self.get_logger().warn(f"Read motor error {name}({motor_id}). Err:{error}", throttle_duration_sec=1.0)
                    # On garde la valeur lue même si erreur moteur (ex: surcharge), mais on le sait
                    raw_positions_list.append(np.int32(value))
                    read_successful = False # Considérer comme échec partiel si erreur moteur ? Oui.
                else:
                    raw_positions_list.append(np.int32(value))

            # Si aucune lecture n'a fonctionné
            if all(v is None for v in raw_positions_list):
                self.get_logger().error("Failed to read any motor position. Skipping cycle.", throttle_duration_sec=2.0)
                return
            # Si certaines lectures ont échoué, on ne peut pas faire FK/IK
            if partial_read:
                 self.get_logger().warn(f"Partial read failure. raw_positions_list={raw_positions_list}. Cannot proceed with FK/IK. Skipping cycle.", throttle_duration_sec=1.0)
                 return

            # Si tout est lu (même avec erreurs moteur potentielles mais valeurs présentes)
            raw_positions_np = np.array(raw_positions_list, dtype=np.int32)

            # --- 2. Appliquer la calibration pour obtenir des degrés ---
            calibrated_angles_deg = apply_calibration(
                raw_positions_np, motors_to_read, self.calibration_data, LEADER_ARM_MOTORS
            )
            if calibrated_angles_deg is None or np.isnan(calibrated_angles_deg).any():
                self.get_logger().error(f"Apply calibration NOK. Raw:{raw_positions_np} -> Calib:{calibrated_angles_deg}. Skip.", throttle_duration_sec=1.0)
                return

            current_pan_deg = calibrated_angles_deg[0]
            current_theta1_deg = calibrated_angles_deg[1] # Lift
            current_theta2_deg = calibrated_angles_deg[2] # Elbow

            # Log état lu (niveau INFO pour l'instant, peut passer en DEBUG)
            self.get_logger().info(f'STATE: Angles Read (deg): Pan={current_pan_deg:.1f}, Lift={current_theta1_deg:.1f}, Elbow={current_theta2_deg:.1f}')

        except Exception as e:
            self.get_logger().error(f"Exception during read/calibration stage: {e}", throttle_duration_sec=1.0)
            import traceback; traceback.print_exc()
            return # Quitter le callback en cas d'exception ici

        # --- 3. Calculer les commandes cibles ---
        # Initialiser les cibles avec les positions actuelles
        target_pan_deg = current_pan_deg
        target_theta1_deg = current_theta1_deg
        target_theta2_deg = current_theta2_deg

        # --- Commande Pan ---
        if abs(signal_pan) >= self.stop_threshold:
            target_pan_deg = current_pan_deg + (self.pan_increment_scale * signal_pan)
            target_pan_deg = np.clip(target_pan_deg, self.pan_min, self.pan_max)
            self.get_logger().debug(f'PAN CMD: Target Pan updated to {target_pan_deg:.1f}°') # DEBUG

        # --- Commande Lift/Elbow (via FK/IK) ---
        # Seulement si le signal de lift est suffisant
        if abs(signal_lift) >= self.stop_threshold:
            try:
                # 3a. Calculer la position actuelle du poignet (Xw, Yw) via FK
                current_xw, current_yw = calculate_fk_wrist(current_theta1_deg, current_theta2_deg)
                self.get_logger().debug(f'FK STATE: Current Wrist (Xw,Yw) = ({current_xw:.3f}, {current_yw:.3f}) m') # DEBUG

                # 3b. Calculer la position cible du poignet
                # Inversion du signe de signal_lift pour le calcul de delta_yw
                delta_yw = -self.yw_increment_scale * signal_lift
                target_yw = current_yw + delta_yw
                target_xw = current_xw # On garde le X actuel

                # Clipping de la cible Yw aux limites de l'espace de travail défini
                target_yw_clipped = np.clip(target_yw, self.Y_MIN_LIMIT, self.Y_MAX_LIMIT)
                if abs(target_yw - target_yw_clipped) > 1e-5:
                    self.get_logger().warn(f"IK Target Yw {target_yw:.3f}m clipped to {target_yw_clipped:.3f}m by Y_LIMITS.", throttle_duration_sec=2.0)
                    target_yw = target_yw_clipped

                self.get_logger().debug(f'IK TARGET: Target Wrist (Xw,Yw) = ({target_xw:.3f}, {target_yw:.3f}) m') # DEBUG

                # 3c. Calculer les angles cibles via IK
                target_theta1_rad, target_theta2_rad, ik_ok = calculate_ik(target_xw, target_yw)

                if ik_ok:
                    # Conversion en degrés
                    _target_th1_deg = math.degrees(target_theta1_rad)
                    _target_th2_deg = math.degrees(target_theta2_rad)

                    # Appliquer les limites articulaires (Joint Limits)
                    target_theta1_deg_clipped = np.clip(_target_th1_deg, self.lift_min, self.lift_max)
                    target_theta2_deg_clipped = np.clip(_target_th2_deg, self.elbow_min, self.elbow_max)

                    # Log si clipping articulaire
                    if abs(target_theta1_deg_clipped - _target_th1_deg) > 0.1 or \
                       abs(target_theta2_deg_clipped - _target_th2_deg) > 0.1:
                         self.get_logger().warn(f"IK angles JOINT clipped: th1={target_theta1_deg_clipped:.1f}(raw:{_target_th1_deg:.1f}), th2={target_theta2_deg_clipped:.1f}(raw:{_target_th2_deg:.1f})", throttle_duration_sec=2.0)

                    # Mettre à jour les angles cibles uniquement si IK ok
                    target_theta1_deg = target_theta1_deg_clipped
                    target_theta2_deg = target_theta2_deg_clipped
                    self.get_logger().debug(f'IK OK: Target Angles Clipped (deg) = ({target_theta1_deg:.1f}, {target_theta2_deg:.1f})') # DEBUG

                else:
                    # Si IK échoue (cible hors portée), ne pas changer les angles Lift/Elbow
                    self.get_logger().warn(f"IK FAILED for target X={target_xw:.3f}, Y={target_yw:.3f}. Holding Lift/Elbow angles.", throttle_duration_sec=1.0)
                    # target_theta1_deg et target_theta2_deg gardent leurs valeurs 'current' initiales

            except Exception as e_ik:
                 self.get_logger().error(f"Exception during FK/IK stage: {e_ik}", throttle_duration_sec=1.0)
                 # En cas d'erreur ici, on ne change pas les angles Lift/Elbow
                 # target_theta1_deg et target_theta2_deg gardent leurs valeurs 'current'

        # --- 4. Envoyer les commandes (après conversion en steps bruts) via SDK ---
        try:
            motors_to_command = ["shoulder_pan", "shoulder_lift", "elbow_flex"]
            # Utiliser les angles cibles (potentiellement modifiés par Pan et/ou IK)
            target_angles_deg_np = np.array([target_pan_deg, target_theta1_deg, target_theta2_deg], dtype=np.float32)

            target_steps = revert_calibration(
                target_angles_deg_np, motors_to_command, self.calibration_data, LEADER_ARM_MOTORS
            )
            if target_steps is None:
                self.get_logger().error("Revert calibration returned None. Skipping write command.")
                return

            # Log final avant écriture (niveau INFO pour visibilité)
            self.get_logger().info(f"COMMAND: Target Angles(deg): {np.round(target_angles_deg_np, 1)} -> Target Steps: {target_steps}")

            addr_goal_pos = SCS_CONTROL_TABLE["Goal_Position"][0]
            write_error_occurred = False
            # Utiliser GroupSyncWrite pour un envoi plus synchronisé ?
            # Pour l'instant, on garde l'écriture individuelle pour simplicité
            for i, name in enumerate(motors_to_command):
                 motor_id = LEADER_ARM_MOTORS[name][0]
                 step_value = target_steps[i].item() # Obtenir la valeur int Python

                 # Utiliser write2ByteTxRx pour envoyer la position Goal_Position
                 scs_comm_result, scs_error = self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, addr_goal_pos, step_value)

                 if scs_comm_result != scs.COMM_SUCCESS:
                      self.get_logger().error(f"Write comm failed {name}({motor_id}). Res:{scs_comm_result}", throttle_duration_sec=1.0)
                      write_error_occurred = True
                 elif scs_error != 0:
                     # Les erreurs moteur pendant l'écriture sont importantes aussi
                     self.get_logger().error(f"Write motor error {name}({motor_id}). Err:{scs_error}", throttle_duration_sec=1.0)
                     write_error_occurred = True

                 # Pause minimale entre écritures individuelles ?
                 # Pourrait aider si le bus est saturé, mais testons sans d'abord.
                 # time.sleep(0.002) # 2ms pause

            if not write_error_occurred:
                 self.get_logger().debug(f"Commands sent successfully for cycle.") # DEBUG

        except Exception as e_write:
            self.get_logger().error(f"Exception during revert_calibration or write stage: {e_write}", throttle_duration_sec=1.0)
            import traceback; traceback.print_exc()


    def destroy_node(self):
        """Nettoyage à l'arrêt du nœud."""
        self.get_logger().info('Shutting down Real Robot Interface node...')
        # Désactiver torque via SDK bas niveau
        if self.port_handler is not None and self.packet_handler is not None and scs is not None:
             try:
                 # Désactiver tous les moteurs définis
                 motors_to_disable = list(LEADER_ARM_MOTORS.keys())
                 addr_torque = SCS_CONTROL_TABLE["Torque_Enable"][0]
                 self.get_logger().info(f"Disabling torque for: {motors_to_disable}")
                 for name in motors_to_disable:
                      motor_id = LEADER_ARM_MOTORS[name][0]
                      try:
                          # Essayer d'envoyer la commande de désactivation torque
                          scs_comm_result, scs_error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, addr_torque, 0)
                          if scs_comm_result != scs.COMM_SUCCESS or scs_error != 0:
                               self.get_logger().warn(f"Could not disable torque for {name} (ID {motor_id}). Res: {scs_comm_result}, Err: {scs_error}")
                      except Exception as e_torque:
                           # Capturer les exceptions potentielles du SDK ici
                           self.get_logger().warn(f"Exception disabling torque for {name}: {e_torque}")
                      time.sleep(0.02) # Petite pause entre chaque désactivation
             except Exception as e:
                 self.get_logger().error(f"Error during torque disable sequence: {e}")

        # Fermer le port via l'instance bus gardée
        if self.leader_bus_instance is not None:
            # Vérifier si la méthode disconnect existe et si l'instance est connectée
            # (ajouter une vérification plus robuste si nécessaire)
            is_connected = getattr(self.leader_bus_instance, 'is_connected', False)
            if is_connected:
                try:
                    self.get_logger().info("Disconnecting leader bus instance...")
                    self.leader_bus_instance.disconnect()
                    self.get_logger().info("Leader bus disconnected.")
                except Exception as e:
                    self.get_logger().error(f"Error during leader bus disconnect: {e}")
            else:
                 self.get_logger().info("Leader bus instance was not connected or status unknown.")
        else:
            # Tentative de fermeture directe si l'objet bus n'existe pas mais le handler oui
             if self.port_handler is not None and hasattr(self.port_handler, 'closePort'):
                  try:
                       self.get_logger().warn("Leader bus object was None, attempting direct port close.")
                       self.port_handler.closePort()
                       self.get_logger().info("Port handler closed directly.")
                  except Exception as e_close:
                       self.get_logger().error(f"Error closing port handler directly: {e_close}")

        super().destroy_node()

# --- Main Entry Point ---
def main(args=None):
    rclpy.init(args=args)
    real_robot_interface_node = None
    node_created_successfully = False
    try:
        # Créer l'instance du nœud
        real_robot_interface_node = RealRobotInterfaceNode()

        # Vérification post-initialisation (un peu redondant avec les exceptions dans init, mais double sécurité)
        if hasattr(real_robot_interface_node, 'port_handler') and \
           real_robot_interface_node.port_handler is not None and \
           hasattr(real_robot_interface_node, 'packet_handler') and \
           real_robot_interface_node.packet_handler is not None and \
           hasattr(real_robot_interface_node, 'calibration_data') and \
           real_robot_interface_node.calibration_data is not None and \
           hasattr(real_robot_interface_node, 'control_signal_subscription'): # Vérifier que le sub a été créé
            node_created_successfully = True
            real_robot_interface_node.get_logger().info("Node components check PASSED post-init.")
        else:
            if real_robot_interface_node is not None:
                 missing = []
                 if not (hasattr(real_robot_interface_node, 'port_handler') and real_robot_interface_node.port_handler): missing.append("Port Handler")
                 if not (hasattr(real_robot_interface_node, 'packet_handler') and real_robot_interface_node.packet_handler): missing.append("Packet Handler")
                 if not (hasattr(real_robot_interface_node, 'calibration_data') and real_robot_interface_node.calibration_data): missing.append("Calibration Data")
                 if not (hasattr(real_robot_interface_node, 'control_signal_subscription')): missing.append("Subscription")
                 real_robot_interface_node.get_logger().fatal(f"Node components check FAILED post-init. Missing: {', '.join(missing)}. Shutting down.")
            else:
                 # Si real_robot_interface_node est None, c'est que __init__ a levé une exception gérée plus bas
                 pass # L'erreur fatale sera loggée dans le bloc except RuntimeError

        # Lancer le spin seulement si l'initialisation a réussi
        if node_created_successfully:
            real_robot_interface_node.get_logger().info("Initialization successful, starting spin loop.")
            rclpy.spin(real_robot_interface_node)
            real_robot_interface_node.get_logger().info("Spin loop exited.") # Log quand spin s'arrête (Ctrl+C ou shutdown)

    except KeyboardInterrupt:
         if real_robot_interface_node is not None: real_robot_interface_node.get_logger().info('Ctrl+C detected, shutting down.')
         else: rclpy.logging.get_logger('real_robot_interface_main').info('Ctrl+C detected during init.')
    except RuntimeError as e:
         # Attraper les erreurs levées par __init__
         rclpy.logging.get_logger('real_robot_interface_main').fatal(f"Runtime error during initialization: {e}")
    except Exception as e:
        # Capturer toute autre exception imprévue
        node_logger = rclpy.logging.get_logger('real_robot_interface_main')
        node_logger.fatal(f'Unhandled exception in main/spin: {e}')
        import traceback
        traceback.print_exc()
    finally:
        # Nettoyage final, s'assurer que le nœud est détruit et rclpy arrêté
        if real_robot_interface_node is not None:
            # Vérifier si le nœud existe et n'est pas déjà détruit
            node_destroyed = True # Assumer détruit par défaut
            try:
                 if real_robot_interface_node: node_destroyed = real_robot_interface_node.is_destroyed()
            except Exception as e_check_destroyed:
                 rclpy.logging.get_logger('real_robot_interface_main').warn(f"Could not check if node is destroyed: {e_check_destroyed}")

            if not node_destroyed:
                 real_robot_interface_node.get_logger().info("Executing final cleanup via destroy_node()...")
                 try:
                      real_robot_interface_node.destroy_node()
                      real_robot_interface_node.get_logger().info("destroy_node() completed.")
                 except Exception as e_destroy:
                      real_robot_interface_node.get_logger().error(f"Exception during destroy_node(): {e_destroy}")

        # Arrêter rclpy s'il est actif
        try:
             if rclpy.ok():
                 rclpy.shutdown()
                 print("rclpy shutdown sequence initiated.") # Utiliser print car logger peut être arrêté
        except Exception as e_shutdown:
             print(f"Error during final rclpy shutdown: {e_shutdown}")
        print("Real Robot Interface main finished.")


if __name__ == '__main__':
    main()
