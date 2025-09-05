import math

from robo_pointer_visual.kinematics import (
    calculate_fk_wrist,
    calculate_ik,
    calculate_gravity_compensation_adj_deg,
    calculate_wrist_angle_for_horizontal,
    calculate_aim_offset,
    L1, L2,
)


def test_fk_oblique_position():
    # Known angles
    t1, t2 = 30.0, 60.0
    x, y = calculate_fk_wrist(t1, t2)
    # Expected using the same formula
    L1, L2 = 0.116, 0.135
    t1r = math.radians(t1)
    t2r = math.radians(t2)
    exp_x = L1 * math.cos(t1r) + L2 * math.cos(t1r + t2r)
    exp_y = L1 * math.sin(t1r) + L2 * math.sin(t1r + t2r)
    assert math.isclose(x, exp_x, rel_tol=1e-9, abs_tol=1e-12)
    assert math.isclose(y, exp_y, rel_tol=1e-9, abs_tol=1e-12)


def test_ik_round_trip_from_fk():
    # Choose a reachable configuration with elbow-up solution
    t1_deg, t2_deg = 40.0, 30.0
    x, y = calculate_fk_wrist(t1_deg, t2_deg)
    th1_rad, th2_rad, ok = calculate_ik(x, y)
    assert ok is True
    assert math.isclose(math.degrees(th1_rad), t1_deg, abs_tol=1e-6)
    assert math.isclose(math.degrees(th2_rad), t2_deg, abs_tol=1e-6)


def test_gravity_compensation_key_angles():
    k2, k3 = 10.0, 5.0
    # q1 = 0° -> cos=1 -> adj2 = k2
    adj2, adj3 = calculate_gravity_compensation_adj_deg(0.0, 0.0, k2, k3)
    assert math.isclose(adj2, k2, abs_tol=1e-9)
    # q1 = 90° -> cos=0 -> adj2 = 0
    adj2, _ = calculate_gravity_compensation_adj_deg(90.0, 0.0, k2, k3)
    assert math.isclose(adj2, 0.0, abs_tol=1e-9)
    # q1 = 180° -> cos=-1 -> adj2 = -k2
    adj2, _ = calculate_gravity_compensation_adj_deg(180.0, 0.0, k2, k3)
    assert math.isclose(adj2, -k2, abs_tol=1e-9)
    # Forearm absolute angle q1+q2 = 90° -> adj3 ≈ 0
    _, adj3 = calculate_gravity_compensation_adj_deg(45.0, 45.0, k2, k3)
    assert abs(adj3) < 1e-9


def test_wrist_angle_keeps_end_effector_horizontal():
    for lift, elbow in [(10.0, 20.0), (60.0, -15.0), (-30.0, 45.0)]:
        wrist = calculate_wrist_angle_for_horizontal(lift, elbow)
        assert math.isclose(wrist, -(lift + elbow), abs_tol=1e-12)


def test_aim_offset_axis_aligned():
    d = 0.05
    # 0°: offset along +X
    dx, dy = calculate_aim_offset(0.0, 30.0, d)
    assert math.isclose(dx, d, abs_tol=1e-9)
    assert math.isclose(dy, 0.0, abs_tol=1e-9)
    # 90°: offset along +Y
    dx, dy = calculate_aim_offset(90.0, 30.0, d)
    assert math.isclose(dx, 0.0, abs_tol=1e-9)
    assert math.isclose(dy, d, abs_tol=1e-9)
    # 180°: offset along -X
    dx, dy = calculate_aim_offset(180.0, 30.0, d)
    assert math.isclose(dx, -d, abs_tol=1e-9)
    assert math.isclose(dy, 0.0, abs_tol=1e-9)


def test_ik_reach_boundaries():
    # Maximum reach (straight arm): D = L1 + L2
    target_x = L1 + L2
    target_y = 0.0
    th1, th2, ok = calculate_ik(target_x, target_y)
    assert ok is True
    # Verify position via FK instead of exact angles (robust to branch/wrap)
    fkx, fky = calculate_fk_wrist(math.degrees(th1), math.degrees(th2))
    assert math.isclose(fkx, target_x, rel_tol=1e-9, abs_tol=1e-9)
    # Near zero, allow a slightly larger absolute tolerance due to acos/clipping
    assert math.isclose(fky, target_y, abs_tol=1e-8)

    # Minimum reach (fully folded): D = |L1 - L2|
    target_x = abs(L1 - L2)
    target_y = 0.0
    th1, th2, ok = calculate_ik(target_x, target_y)
    assert ok is True
    fkx, fky = calculate_fk_wrist(math.degrees(th1), math.degrees(th2))
    assert math.isclose(fkx, target_x, rel_tol=1e-9, abs_tol=1e-9)
    assert math.isclose(fky, target_y, abs_tol=1e-8)


def test_ik_out_of_reach_boundaries():
    # Slightly beyond maximum reach should fail
    eps = 2e-6
    target_x = L1 + L2 + eps
    target_y = 0.0
    _, _, ok = calculate_ik(target_x, target_y)
    assert ok is False

    # Slightly within maximum reach should succeed
    target_x = L1 + L2 - eps
    target_y = 0.0
    _, _, ok = calculate_ik(target_x, target_y)
    assert ok is True


def test_ik_round_trip_random_samples():
    # Deterministic seed
    import random
    random.seed(42)
    for _ in range(5):
        t1_deg = random.uniform(-60.0, 60.0)
        t2_deg = random.uniform(-60.0, 60.0)
        x, y = calculate_fk_wrist(t1_deg, t2_deg)
        th1, th2, ok = calculate_ik(x, y)
        assert ok is True
        fx, fy = calculate_fk_wrist(math.degrees(th1), math.degrees(th2))
        assert math.isclose(fx, x, rel_tol=1e-9, abs_tol=1e-9)
        assert math.isclose(fy, y, rel_tol=1e-9, abs_tol=1e-9)


def test_ik_zero_distance_unhandled():
    # At exactly 0 distance, IK should report failure per implementation guard
    _, _, ok = calculate_ik(0.0, 0.0)
    assert ok is False
