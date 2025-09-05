import numpy as np
import math

from robo_pointer_visual.real_robot_interface import (
    apply_calibration,
    revert_calibration,
    MODEL_RESOLUTION,
)


def test_linear_mode_basic_and_roundtrip():
    # Calibration describing a linear actuator from step 1000 to 3000
    calib = {
        "motor_names": ["linear_joint"],
        "calib_mode": ["LINEAR"],
        "drive_mode": [0],
        "homing_offset": [0],
        "start_pos": [1000],
        "end_pos": [3000],
    }
    motors_def = {"linear_joint": [1, "sts3215"]}

    # apply_calibration: raw -> percentage in [0,100], with clipping [-10,110]
    raw = np.array([1000, 2000, 3000, 50, 4000])
    vals = apply_calibration(raw, ["linear_joint"] * len(raw), calib, motors_def)
    # First three are 0%, 50%, 100%
    assert np.isclose(vals[0], 0.0)
    assert np.isclose(vals[1], 50.0)
    assert np.isclose(vals[2], 100.0)
    # Below/above -> clipped to [-10, 110]
    assert np.isclose(vals[3], -10.0)
    assert np.isclose(vals[4], 110.0)

    # revert_calibration: percentage -> raw steps
    targets = np.array([0.0, 50.0, 100.0])
    steps = revert_calibration(targets, ["linear_joint"] * len(targets), calib, motors_def)
    assert steps[0] == 1000
    assert steps[1] == 2000
    assert steps[2] == 3000


def test_linear_mode_degenerate_range():
    # start_pos == end_pos -> apply -> 50.0, revert -> start_pos
    calib = {
        "motor_names": ["linear_joint"],
        "calib_mode": ["LINEAR"],
        "drive_mode": [0],
        "homing_offset": [0],
        "start_pos": [1500],
        "end_pos": [1500],
    }
    motors_def = {"linear_joint": [1, "sts3215"]}

    raw = np.array([1500])
    vals = apply_calibration(raw, ["linear_joint"], calib, motors_def)
    assert np.isclose(vals[0], 50.0)

    steps = revert_calibration(np.array([0.0, 50.0, 100.0]), ["linear_joint"] * 3, calib, motors_def)
    # Always clamped to start_pos/end_pos
    assert steps[0] == 1500
    assert steps[1] == 1500
    assert steps[2] == 1500


def test_unknown_motor_behaviour():
    calib = {
        "motor_names": ["known_joint"],
        "calib_mode": ["LINEAR"],
        "drive_mode": [0],
        "homing_offset": [0],
        "start_pos": [0],
        "end_pos": [4095],
    }
    motors_def = {"known_joint": [1, "sts3215"], "unknown_joint": [2, "sts3215"]}

    # apply_calibration returns NaN for unknown motor
    vals = apply_calibration(np.array([0]), ["unknown_joint"], calib, motors_def)
    assert np.isnan(vals[0])

    # revert_calibration returns default step = resolution/2 for unknown motor
    steps = revert_calibration(np.array([0.0]), ["unknown_joint"], calib, motors_def)
    assert steps[0] == MODEL_RESOLUTION["sts3215"] // 2


def test_degree_mode_apply_wraparound_near_minus_180():
    # Verify apply_calibration mapping around wrap boundary using homing_offset=0
    calib = {
        "motor_names": ["rot_joint"],
        "calib_mode": ["DEGREE"],
        "drive_mode": [0],
        "homing_offset": [0],
        "start_pos": [0],
        "end_pos": [4095],
    }
    motors_def = {"rot_joint": [1, "sts3215"]}
    res = MODEL_RESOLUTION["sts3215"]
    steps_per_180 = res / 2.0
    # A raw value slightly above steps_per_180 should wrap to a negative angle near -180°
    raw = np.array([int(steps_per_180 + 11)])
    vals = apply_calibration(raw, ["rot_joint"], calib, motors_def)
    assert vals[0] < 0.0  # negative due to wrap
    # magnitude close to -180 + small positive delta
    expected_deg = -180.0 + (11 / steps_per_180) * 180.0
    assert math.isclose(vals[0], expected_deg, rel_tol=1e-3, abs_tol=1e-2)
