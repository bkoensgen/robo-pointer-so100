import math
import numpy as np
from typing import Tuple, Optional

# --- Constantes (Longueurs des segments en mètres) ---
L1: float = 0.116
L2: float = 0.135

# --- Fonctions Cinématiques ---

def calculate_fk_wrist(theta1_deg: float, theta2_deg: float) -> Tuple[float, float]:
    """Calcule la cinématique directe pour la position du poignet."""
    theta1_rad = math.radians(theta1_deg)
    theta2_rad = math.radians(theta2_deg)
    wrist_x = L1 * math.cos(theta1_rad) + L2 * math.cos(theta1_rad + theta2_rad)
    wrist_y = L1 * math.sin(theta1_rad) + L2 * math.sin(theta1_rad + theta2_rad)
    return wrist_x, wrist_y

def calculate_ik(target_xw: float, target_yw: float) -> Tuple[Optional[float], Optional[float], bool]:
    """Calcule la cinématique inverse pour atteindre une cible cartésienne."""
    TOLERANCE_SQ = 1e-12
    TOLERANCE_REACH = 1e-6
    try:
        D_sq = target_xw**2 + target_yw**2
        if D_sq < TOLERANCE_SQ: D = 0.0
        else: D = math.sqrt(D_sq)
        limit_min = abs(L1 - L2)
        limit_max = L1 + L2
        if D < limit_min - TOLERANCE_REACH or D > limit_max + TOLERANCE_REACH: return None, None, False
        if abs(L1) < TOLERANCE_SQ or abs(L2) < TOLERANCE_SQ: return None, None, False
        cos_gamma = (L1**2 + L2**2 - D_sq) / (2 * L1 * L2)
        gamma = math.acos(np.clip(cos_gamma, -1.0, 1.0))
        theta2_rad = math.pi - gamma
        alpha = math.atan2(target_yw, target_xw)
        if D < TOLERANCE_SQ: return None, None, False
        cos_beta = (L1**2 + D_sq - L2**2) / (2 * L1 * D)
        beta = math.acos(np.clip(cos_beta, -1.0, 1.0))
        theta1_rad = alpha - beta
        return theta1_rad, theta2_rad, True
    except (ValueError, Exception):
        return None, None, False

def calculate_gravity_compensation_adj_deg(
    current_theta1_deg: float, current_theta2_deg: float, k_gravity_id2: float, k_gravity_id3: float
) -> Tuple[float, float]:
    """Calcule l'ajustement des angles pour compenser l'effet de la gravité."""
    q1_rad = math.radians(current_theta1_deg)
    q2_rad = math.radians(current_theta2_deg)
    gravity_adj_id2_deg = k_gravity_id2 * math.cos(q1_rad)
    absolute_forearm_angle_rad = q1_rad + q2_rad
    gravity_adj_id3_deg = k_gravity_id3 * math.cos(absolute_forearm_angle_rad)
    return gravity_adj_id2_deg, gravity_adj_id3_deg

def calculate_wrist_angle_for_horizontal(lift_angle_deg: float, elbow_angle_deg: float) -> float:
    """Calcule l'angle du poignet nécessaire pour qu'il reste à l'horizontale."""
    return - (lift_angle_deg + elbow_angle_deg)

def calculate_aim_offset(
    final_arm_angle_deg: float, camera_tilt_deg: float, camera_offset_m: float
) -> Tuple[float, float]:
    """
    Calcule le décalage (dx, dy) à appliquer à la cible IK pour compenser
    le déport et l'inclinaison de la caméra par rapport à l'axe du poignet.

    Args:
        final_arm_angle_deg: L'angle d'orientation final du bras (et de la pince) en degrés.
        camera_tilt_deg: L'angle d'inclinaison de la caméra vers le bas par rapport au bras (positif).
        camera_offset_m: La distance en avant entre l'axe du poignet et la caméra.

    Returns:
        Un tuple (offset_x, offset_y) à soustraire de la cible cartésienne.
    """
    final_arm_angle_rad = math.radians(final_arm_angle_deg)
    cos_arm = math.cos(final_arm_angle_rad)
    sin_arm = math.sin(final_arm_angle_rad)

    offset_x_base_frame = camera_offset_m * cos_arm
    offset_y_base_frame = camera_offset_m * sin_arm

    return offset_x_base_frame, offset_y_base_frame
