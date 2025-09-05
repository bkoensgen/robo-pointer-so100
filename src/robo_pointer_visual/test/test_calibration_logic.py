import numpy as np
from robo_pointer_visual.real_robot_interface import revert_calibration, apply_calibration

# Créer un faux dictionnaire de calibration pour les tests
FAKE_CALIB_DATA = {
    "homing_offset": [2048],
    "drive_mode": [0],
    "start_pos": [0],
    "end_pos": [4095],
    "calib_mode": ["DEGREE"],
    "motor_names": ["shoulder_lift"]
}
FAKE_MOTORS_DEF = {"shoulder_lift": [1, "sts3215"]}

def test_revert_calibration_zero_deg():
    """Vérifie que 0 degré donne un step valide (clampé si nécessaire)."""
    target_angle_deg = np.array([0.0])
    motor_name = ["shoulder_lift"]
    
    steps = revert_calibration(target_angle_deg, motor_name, FAKE_CALIB_DATA, FAKE_MOTORS_DEF)
    
    # Avec homing_offset=2048 et plage [0..4095], la conversion est clampée à start_pos
    assert steps[0] == FAKE_CALIB_DATA["start_pos"][0]

def test_apply_revert_calibration_cycle():
    """Teste que la conversion degré -> step -> degré revient au point de départ (calibration centrée)."""
    # Utiliser une calibration centrée (homing_offset=0) pour éviter les clamps
    CYCLE_CALIB = {
        "homing_offset": [0],
        "drive_mode": [0],
        "start_pos": [0],
        "end_pos": [4095],
        "calib_mode": ["DEGREE"],
        "motor_names": ["shoulder_lift"],
    }
    original_angle_deg = np.array([30.0])
    motor_name = ["shoulder_lift"]

    steps = revert_calibration(original_angle_deg, motor_name, CYCLE_CALIB, FAKE_MOTORS_DEF)
    final_angle_deg = apply_calibration(steps, motor_name, CYCLE_CALIB, FAKE_MOTORS_DEF)
    # Quantization to integer steps introduces ~0.03° error on 4096-step encoders
    assert np.isclose(final_angle_deg[0], 30.0, atol=0.1)
