import math
from robo_pointer_visual.kinematics import calculate_fk_wrist, calculate_ik

# On importe pytest pour les fonctions avancées, mais pas obligatoire au début
import pytest

def test_fk_simple_positions():
    """Teste la cinématique directe dans des positions simples et connues."""
    # Test 1: Bras tendu à l'horizontale (0, 0 degrés)
    x, y = calculate_fk_wrist(0.0, 0.0)
    assert math.isclose(x, 0.116 + 0.135) # L1 + L2
    assert math.isclose(y, 0.0)

    # Test 2: Bras à la verticale (+90, 0 degrés)
    x, y = calculate_fk_wrist(90.0, 0.0)
    assert math.isclose(x, 0.0)
    assert math.isclose(y, 0.116 + 0.135) # L1 + L2

def test_ik_simple_positions():
    """Teste que la cinématique inverse retrouve bien les angles des positions simples."""
    # Test 1: Cible horizontale
    target_x = 0.116 + 0.135
    target_y = 0.0
    theta1_rad, theta2_rad, ok = calculate_ik(target_x, target_y)
    assert ok is True
    assert math.isclose(math.degrees(theta1_rad), 0.0)
    assert math.isclose(math.degrees(theta2_rad), 0.0)

def test_ik_out_of_reach():
    """Teste que l'IK échoue correctement quand la cible est trop loin."""
    target_x = 10.0 # Clairement hors de portée
    target_y = 10.0
    theta1_rad, theta2_rad, ok = calculate_ik(target_x, target_y)
    assert ok is False
