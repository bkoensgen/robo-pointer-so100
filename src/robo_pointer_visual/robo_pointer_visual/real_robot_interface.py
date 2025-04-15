# Fichier: ~/ros2_ws/src/robo_pointer_visual/robo_pointer_visual/real_robot_interface.py

import numpy as np
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus, FeetechMotorsBusConfig

from .kinematics import calculate_fk_wrist, calculate_ik

LEADER_ARM_PORT = "/dev/robot_arm"

LEADER_ARM_MOTORS = {
    "shoulder_pan": [1, "sts3215"],
    "shoulder_lift": [2, "sts3215"],
    "elbow_flex": [3, "sts3215"],
    "wrist_flex": [4, "sts3215"], # Non utilisé pour l'instant
    "wrist_roll": [5, "sts3215"], # Non utilisé pour l'instant
    "gripper": [6, "sts3215"],    # Non utilisé pour l'instant
}

# Calibration pour les moteurs utilisés (Pan, Lift, Elbow)
# DEFAULT_CALIBRATION = {
#     "motor_names": ["shoulder_pan", "shoulder_lift", "elbow_flex"],
#     "calib_mode": ["DEGREE", "DEGREE", "DEGREE"],
#     "drive_mode": [0, 0, 0], # Ajuster si la rotation physique est inversée
#     "homing_offset": [2048, 2048, 2048], # Position brute (0-4095) pour 0 degré. À VÉRIFIER/CALIBRER.
#     "start_pos": [0, 0, 0], # Non utilisé pour DEGREE mode
#     "end_pos": [0, 0, 0]  # Non utilisé pour DEGREE mode
# }


class RealRobotInterfaceNode(Node):
    """
    Interface avec le bras robotique Feetech réel.
    - S'abonne à /robot_control_signal (Vector3) pour les commandes Pan/Lift.
    - Contrôle le Pan (ID 1) directement avec un delta angulaire.
    - Contrôle le Lift (ID 2 & 3 coordonnés) en utilisant FK/IK pour atteindre
      une position Y cible basée sur le signal de lift.
    - Lit les positions actuelles et écrit les Goal_Position aux moteurs via FeetechMotorsBus.
    """
    def __init__(self):
        super().__init__('real_robot_interface')
        self.get_logger().info('Real Robot Interface node starting...')

        self.leader_bus = None
        try:
            leader_config = FeetechMotorsBusConfig(port=LEADER_ARM_PORT, motors=LEADER_ARM_MOTORS)
            self.leader_bus = FeetechMotorsBus(leader_config)
            self.leader_bus.connect()
            self.get_logger().info(f"Successfully connected to leader arm bus on {LEADER_ARM_PORT}")
            #self.leader_bus.set_calibration(DEFAULT_CALIBRATION)
            #self.get_logger().info("Default calibration profile set for Pan, Lift, Elbow motors.")
        except Exception as e:
            self.get_logger().fatal(f"Failed to connect/setup leader arm bus on {LEADER_ARM_PORT}: {e}")
            self.leader_bus = None

        # --- Paramètres ROS 2 ---
        # Contrôle Pan (ID 1)
        self.declare_parameter('pan_increment_scale', 0.1) # Facteur pour convertir signal Pan -> delta angle Pan
        self.pan_increment_scale = self.get_parameter('pan_increment_scale').get_parameter_value().double_value
        self.declare_parameter('pan_angle_min', -110.0) # Limite min angle Pan (degrés)
        self.declare_parameter('pan_angle_max', 110.0) # Limite max angle Pan (degrés)
        self.pan_min = self.get_parameter('pan_angle_min').get_parameter_value().double_value
        self.pan_max = self.get_parameter('pan_angle_max').get_parameter_value().double_value

        # Contrôle Lift (ID 2 & 3 via IK)
        # *** ATTENTION: Valeur par défaut très petite recommandée ***
        self.declare_parameter('yw_increment_scale', 0.0005) # Facteur pour convertir signal Lift -> delta Y (mètres)
        self.yw_increment_scale = self.get_parameter('yw_increment_scale').get_parameter_value().double_value
        self.declare_parameter('lift_angle_min', -90.0)  # Limite min angle Lift (ID 2) (degrés)
        self.declare_parameter('lift_angle_max', 90.0)   # Limite max angle Lift (ID 2) (degrés)
        self.lift_min = self.get_parameter('lift_angle_min').get_parameter_value().double_value
        self.lift_max = self.get_parameter('lift_angle_max').get_parameter_value().double_value

        # *** ATTENTION: Vérifier la limite max physique du coude ***
        self.declare_parameter('elbow_angle_min', 0.0)   # Limite min angle Elbow (ID 3) (degrés, 0=étendu)
        self.declare_parameter('elbow_angle_max', 150.0) # Limite max angle Elbow (ID 3) (degrés, ex: 150=max flexion) - À VÉRIFIER
        self.elbow_min = self.get_parameter('elbow_angle_min').get_parameter_value().double_value
        self.elbow_max = self.get_parameter('elbow_angle_max').get_parameter_value().double_value

        # Contrôle Général
        self.declare_parameter('stop_threshold', 0.01) # Seuil signal (Pan ou Lift) sous lequel on ne bouge pas
        self.stop_threshold = self.get_parameter('stop_threshold').get_parameter_value().double_value

        self.get_logger().info(f"Pan control: Scale={self.pan_increment_scale:.4f}, Limits=[{self.pan_min:.1f}, {self.pan_max:.1f}] deg")
        self.get_logger().info(f"Lift control: YW_Scale={self.yw_increment_scale:.4f}")
        self.get_logger().info(f"Joint Limits: Lift=[{self.lift_min:.1f}, {self.lift_max:.1f}] deg, Elbow=[{self.elbow_min:.1f}, {self.elbow_max:.1f}] deg")
        self.get_logger().info(f"Stop Threshold: {self.stop_threshold:.3f}")

        # --- Subscriber ---
        self.control_signal_subscription = self.create_subscription(
            Vector3,
            '/robot_control_signal',
            self.control_signal_callback,
            10 # QoS profile depth
        )
        self.get_logger().info('Subscribed to /robot_control_signal')

    def control_signal_callback(self, msg: Vector3):
        """Traite un message de contrôle Vector3 et commande les moteurs."""
        if self.leader_bus is None or not self.leader_bus.is_connected:
            self.get_logger().warn("Leader bus not connected, skipping command.", throttle_duration_sec=5)
            return

        signal_pan = msg.x
        signal_lift = msg.y
        self.get_logger().info(f'CALLBACK: Received Signal: Pan={signal_pan:.3f}, Lift={signal_lift:.3f} | StopThreshold={self.stop_threshold:.3f}')

        # --- Shoulder Pan Control (ID 1) ---
        if abs(signal_pan) >= self.stop_threshold:
            delta_angle_pan = self.pan_increment_scale * signal_pan
            try:
                motor_name_pan = "shoulder_pan"
                current_pan_deg = self.leader_bus.read("Present_Position", motor_name_pan).item()
                self.get_logger().debug(f"Read current Pan: {current_pan_deg:.1f} deg")

                target_pan_deg = current_pan_deg + delta_angle_pan
                target_pan_clipped_deg = np.clip(target_pan_deg, self.pan_min, self.pan_max)

                if abs(target_pan_deg - target_pan_clipped_deg) > 0.1:
                    self.get_logger().warn(f"Pan target {target_pan_deg:.1f} clipped to {target_pan_clipped_deg:.1f} deg")

                self.get_logger().info(f"PAN Command: Curr={current_pan_deg:.1f}, Sig={signal_pan:.2f} -> Target={target_pan_clipped_deg:.1f} deg")
                self.leader_bus.write("Goal_Position", target_pan_clipped_deg, motor_name_pan)

            except Exception as e:
                self.get_logger().error(f"Failed to command {motor_name_pan}: {e}", throttle_duration_sec=1.0)
        else:
             self.get_logger().debug(f"Pan signal {signal_pan:.3f} below threshold, holding Pan.")

        # --- Shoulder Lift & Elbow Flex Control (ID 2 & 3 via FK/IK) ---
        if abs(signal_lift) >= self.stop_threshold:
            self.get_logger().info(f'LIFT_CTRL: Signal {signal_lift:.3f} >= threshold {self.stop_threshold:.3f}. Entering FK/IK block.') # LOG 2a
            current_theta1_deg = None
            current_theta2_deg = None
            try:
                # Essayer de lire les DEUX positions nécessaires pour FK/IK
                current_theta1_deg = self.leader_bus.read("Present_Position", "shoulder_lift").item()
                current_theta2_deg = self.leader_bus.read("Present_Position", "elbow_flex").item()
                # LOG 3: Voir les angles lus (maintenant calibrés !)
                self.get_logger().info(f'LIFT_CTRL: Read positions: Lift={current_theta1_deg:.1f}, Elbow={current_theta2_deg:.1f} deg')

            except Exception as e:
                # LOG 4: Voir si la lecture échoue
                self.get_logger().error(f"LIFT_CTRL: Failed to read Lift/Elbow position for FK/IK: {e}", throttle_duration_sec=1.0)
                return # Sortir du callback pour ce cycle

            # Si les lectures sont OK, continuer avec FK/IK
            try:
                # 1. Calculer la position actuelle du poignet (FK)
                current_xw, current_yw = calculate_fk_wrist(current_theta1_deg, current_theta2_deg)
                # LOG 5: Voir la position FK actuelle
                self.get_logger().info(f"LIFT_CTRL: FK result: Current Xw={current_xw:.3f}, Yw={current_yw:.3f} m")

                # 2. Calculer le déplacement Y souhaité et la position Y cible
                delta_yw = self.yw_increment_scale * signal_lift
                target_yw = current_yw + delta_yw
                target_xw = current_xw # Garde la même position X
                # LOG 6: Voir la cible calculée
                self.get_logger().info(f"LIFT_CTRL: Target Coords: Xw={target_xw:.3f}, Yw={target_yw:.3f} m (delta_yw={delta_yw:.4f})")

                Y_MIN_LIMIT = -0.02 
                Y_MAX_LIMIT = 0.20
                # -------------------------------------------
                target_yw_clipped = np.clip(target_yw, Y_MIN_LIMIT, Y_MAX_LIMIT)
                if abs(target_yw - target_yw_clipped) > 1e-4:
                    self.get_logger().warn(f"LIFT_CTRL: Target Yw {target_yw:.3f} clipped to {target_yw_clipped:.3f} m.")
                    target_yw = target_yw_clipped # Utiliser la valeur clippée

                # 3. Calculer les angles cibles (IK) - Utilise la cible potentiellement clippée
                self.get_logger().info(f"LIFT_CTRL: Calling IK with clipped target: Xw={target_xw:.3f}, Yw={target_yw:.3f} m") # Log de la cible envoyée à IK
                target_theta1_rad, target_theta2_rad, ik_ok = calculate_ik(target_xw, target_yw)

                # 4. Si IK réussit, convertir, clipper et commander
                if ik_ok:
                    # LOG 7: Confirmer que l'IK a réussi
                    self.get_logger().info(f"LIFT_CTRL: IK SUCCESS!")
                    target_theta1_deg = math.degrees(target_theta1_rad)
                    target_theta2_deg = math.degrees(target_theta2_rad)
                    self.get_logger().debug(f"IK result: Target th1={target_theta1_deg:.1f}, th2={target_theta2_deg:.1f} deg") # debug ok ici

                    # Appliquer les limites articulaires
                    target_theta1_clipped_deg = np.clip(target_theta1_deg, self.lift_min, self.lift_max)
                    target_theta2_clipped_deg = np.clip(target_theta2_deg, self.elbow_min, self.elbow_max)

                    if abs(target_theta1_deg - target_theta1_clipped_deg) > 0.1 or \
                       abs(target_theta2_deg - target_theta2_clipped_deg) > 0.1:
                        self.get_logger().warn(f"IK angles clipped: th1={target_theta1_clipped_deg:.1f} (was {target_theta1_deg:.1f}), "
                                             f"th2={target_theta2_clipped_deg:.1f} (was {target_theta2_deg:.1f}) deg")

                    # Calculer la différence entre la cible clippée et la position lue au début du cycle
                    delta_cmd_th1 = target_theta1_clipped_deg - current_theta1_deg
                    delta_cmd_th2 = target_theta2_clipped_deg - current_theta2_deg
                    self.get_logger().info(f"LIFT_CTRL: Angle Diffs Cmd: dTh1={delta_cmd_th1:.1f}, dTh2={delta_cmd_th2:.1f} deg")

                    # LOG 8: Voir la commande finale envoyée
                    self.get_logger().info(f"LIFT_CTRL: WRITE Command -> th1={target_theta1_clipped_deg:.1f}, th2={target_theta2_clipped_deg:.1f} deg")
                    self.leader_bus.write("Goal_Position", target_theta1_clipped_deg, "shoulder_lift")
                    self.leader_bus.write("Goal_Position", target_theta2_clipped_deg, "elbow_flex")

                else:
                    # LOG 9: Voir si l'IK échoue explicitement
                    self.get_logger().warn(f"LIFT_CTRL: IK FAILED for target X={target_xw:.3f}, Y={target_yw:.3f} from angles ({current_theta1_deg:.1f}, {current_theta2_deg:.1f})", throttle_duration_sec=1.0)

            except Exception as e:
                 # LOG 10: Voir toute autre erreur dans le bloc FK/IK
                 self.get_logger().error(f"LIFT_CTRL: Error during FK/IK calculation or command: {e}", throttle_duration_sec=1.0)

        else:
             # LOG 11: Confirmer qu'on n'a PAS dépassé le seuil
             self.get_logger().info(f"LIFT_CTRL: Signal {signal_lift:.3f} < threshold {self.stop_threshold:.3f}. Holding Lift/Elbow.") # LOG 2b


    def destroy_node(self):
        """Nettoyage à l'arrêt du nœud."""
        self.get_logger().info('Shutting down Real Robot Interface node...')
        if self.leader_bus is not None and self.leader_bus.is_connected:
            try:
                # Optionnel: Mettre les moteurs en position de repos ou désactiver le torque
                # Exemple: Envoyer à une position initiale sûre?
                # self.leader_bus.write(...)
                # Ou juste désactiver le torque pour qu'ils soient libres
                # self.leader_bus.write("Torque_Enable", [0, 0, 0], ["shoulder_pan", "shoulder_lift", "elbow_flex"])
                # self.get_logger().info("Torque disabled for Pan, Lift, Elbow motors.")
                self.leader_bus.disconnect()
                self.get_logger().info("Leader bus disconnected.")
            except Exception as e:
                self.get_logger().error(f"Error during leader bus cleanup: {e}")
        super().destroy_node()

# --- Main Entry Point ---
def main(args=None):
    rclpy.init(args=args)
    real_robot_interface_node = None
    try:
        real_robot_interface_node = RealRobotInterfaceNode()
        # Vérifier si la connexion au bus a réussi avant de spinner
        if real_robot_interface_node.leader_bus is not None and real_robot_interface_node.leader_bus.is_connected:
            rclpy.spin(real_robot_interface_node)
        else:
            # Si la connexion a échoué dans __init__, le noeud existe mais n'est pas fonctionnel
            real_robot_interface_node.get_logger().fatal("Initialization failed (bus not connected). Shutting down without spinning.")
            # On force la destruction ici car spin() n'a pas été appelé
            if real_robot_interface_node is not None:
                real_robot_interface_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    except KeyboardInterrupt:
        if real_robot_interface_node:
            real_robot_interface_node.get_logger().info('Ctrl+C detected, shutting down interface node.')
        # Le bloc finally s'occupera du cleanup
    except Exception as e:
        # Log toute autre erreur inattendue
        node_logger = rclpy.logging.get_logger('real_robot_interface_main')
        node_logger.fatal(f'Unhandled exception in main: {e}')
        import traceback
        traceback.print_exc() # Imprime la trace complète pour le débogage
    finally:
        # Assurer le nettoyage même si spin n'a pas été appelé ou a échoué
        if real_robot_interface_node is not None and not real_robot_interface_node.is_destroyed():
             # Vérifier si le nœud n'a pas déjà été détruit (ex: échec init)
             real_robot_interface_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            print("Interface node shutdown complete.")


if __name__ == '__main__':
    main()
