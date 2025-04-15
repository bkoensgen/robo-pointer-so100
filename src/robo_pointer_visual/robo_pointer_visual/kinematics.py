# Fichier: ~/ros2_ws/src/robo_pointer_visual/robo_pointer_visual/kinematics.py

import math
import numpy as np
from typing import Tuple, Optional

# --- Constantes (Longueurs des segments en mètres) ---
L1: float = 0.116  # Longueur épaule (axe 2) -> coude (axe 3)
L2: float = 0.135  # Longueur coude (axe 3) -> poignet (axe 4)

# --- Fonctions Cinématiques ---

def calculate_fk_wrist(theta1_deg: float, theta2_deg: float) -> Tuple[float, float]:
    """
    Calcule la Cinématique Directe (FK) pour le poignet d'un bras planaire 2-DOF.

    Args:
        theta1_deg: Angle de l'épaule (ID 2) en degrés.
        theta2_deg: Angle du coude (ID 3) en degrés (convention 0=étendu, 180=replié).

    Returns:
        Tuple[float, float]: Coordonnées cartésiennes (Xw, Yw) du poignet en mètres.
    """
    theta1_rad = math.radians(theta1_deg)
    theta2_rad = math.radians(theta2_deg) # Angle de flexion

    # Formule FK standard utilisant l'angle relatif (qui correspond à notre theta2_rad)
    wrist_x = L1 * math.cos(theta1_rad) + L2 * math.cos(theta1_rad + theta2_rad)
    wrist_y = L1 * math.sin(theta1_rad) + L2 * math.sin(theta1_rad + theta2_rad)

    return wrist_x, wrist_y

def calculate_ik(target_xw: float, target_yw: float) -> Tuple[Optional[float], Optional[float], bool]:
    """
    Calcule la Cinématique Inverse (IK) pour un bras planaire 2-DOF.

    Args:
        target_xw: Coordonnée X souhaitée pour le poignet (mètres).
        target_yw: Coordonnée Y souhaitée pour le poignet (mètres).

    Returns:
        Tuple[Optional[float], Optional[float], bool]:
            (theta1_rad, theta2_rad, success)
            Angles en radians ou None si inatteignable.
            Success est True si une solution est trouvée.
    """
    theta1_rad: Optional[float] = None
    theta2_rad: Optional[float] = None
    success: bool = False
    TOLERANCE_SQ: float = 1e-12 # Tolérance pour D² proche de zéro
    TOLERANCE_REACH: float = 1e-6 # Tolérance pour la vérification d'atteignabilité

    try:
        # 1. Calcul de D² et D
        D_sq = target_xw**2 + target_yw**2
        if D_sq < -TOLERANCE_SQ: # Devrait être >= 0
             # print(f"Erreur IK: D_sq négatif? {D_sq:.3g}") # Log retiré
             return None, None, False

        if D_sq < TOLERANCE_SQ:
            # Cible à l'origine (ou très proche) - singularité
            # print("Avertissement IK: Cible très proche de l'origine (singularité).") # Log retiré
            D = 0.0
        else:
            D = math.sqrt(D_sq)

        # 2. Vérification d'atteignabilité
        limit_min = abs(L1 - L2)
        limit_max = L1 + L2
        if D < limit_min - TOLERANCE_REACH or D > limit_max + TOLERANCE_REACH:
            # print(f"Debug IK: Hors portée. D={D:.3f}m. Limites=[{limit_min:.3f}, {limit_max:.3f}]m") # Log retiré
            return None, None, False

        # Vérification division par zéro potentielle (si L1 ou L2 sont nuls)
        if abs(L1) < TOLERANCE_SQ or abs(L2) < TOLERANCE_SQ:
             # print("Erreur IK: Longueurs L1 ou L2 nulles.") # Log retiré
             return None, None, False

        # 3. Calcul de theta2 (via angle gamma dans le triangle)
        cos_gamma = (L1**2 + L2**2 - D_sq) / (2 * L1 * L2)
        cos_gamma_clipped = np.clip(cos_gamma, -1.0, 1.0)
        gamma = math.acos(cos_gamma_clipped) # Angle triangle coude (0=replié, pi=étendu)
        theta2_rad = math.pi - gamma # Convention classique (0=étendu, pi=replié)

        # 4. Calcul de theta1 (via angles alpha et beta)
        alpha = math.atan2(target_yw, target_xw) # Angle ligne Base-Poignet / Axe X

        # Vérification division par zéro potentielle pour beta (si D est nul)
        if D < TOLERANCE_SQ: # Normalement déjà géré par la vérif d'atteignabilité sauf si L1=L2
             # print("Erreur IK: Distance D nulle pour calcul beta.") # Log retiré
             # Si L1=L2 et D=0, la cible est à l'origine, theta2=pi, theta1 indéfini.
             # Retourner un échec ici est plus sûr qu'une solution arbitraire.
             return None, None, False

        cos_beta = (L1**2 + D_sq - L2**2) / (2 * L1 * D)
        cos_beta_clipped = np.clip(cos_beta, -1.0, 1.0)
        beta = math.acos(cos_beta_clipped) # Angle entre L1 et ligne Base-Poignet

        # Solution "coude vers le haut"
        theta1_rad = alpha - beta

        # 5. Vérification des limites articulaires (Optionnel mais recommandé)
        # TODO: Ajouter ici la vérification des limites physiques des moteurs si nécessaire

        success = True

    except ValueError as ve:
        print(f"Erreur mathématique dans calculate_ik: {ve}") # Gardé car exceptionnel
        return None, None, False
    except Exception as e:
        print(f"Erreur inattendue dans calculate_ik: {e}") # Gardé car exceptionnel
        return None, None, False

    return theta1_rad, theta2_rad, success

# --- Bloc de Test ---
if __name__ == "__main__":
    print("Test de la fonction calculate_ik et verification avec calculate_fk...")

    def check_close(val1: float, val2: float, tol: float = 1e-6) -> bool:
        return abs(val1 - val2) < tol

    # Définition des cas de test (identiques à avant)
    test_cases = [
        {"id": 1, "x": L1 + L2, "y": 0.0, "th1_exp": 0.0, "th2_exp": 0.0, "ok_exp": True},
        {"id": 2, "x": 0.0, "y": L1 + L2, "th1_exp": 90.0, "th2_exp": 0.0, "ok_exp": True},
        {"id": 3, "x": L1, "y": 0.0, "ok_exp": True},
        {"id": 4, "x": L1 + L2 + 0.1, "y": 0.0, "ok_exp": False},
        {"id": 5, "x": 0.150, "y": 0.100, "ok_exp": True},
        {"id": 6, "x": abs(L1 - L2) * 0.5, "y": 0.0, "ok_exp": abs(L1 - L2) < 1e-9}, # Échec sauf si L1=L2
        {"id": 7, "x": L1 * math.cos(math.radians(45)), "y": L1 * math.sin(math.radians(45)), "ok_exp": True}
    ]

    # Pré-calcul pour cas 3
    if abs(L1) > 1e-9 and abs(L2) > 1e-9:
        D3 = L1
        if D3 >= abs(L1 - L2) - 1e-6 and D3 <= L1 + L2 + 1e-6 : # Vérif atteignabilité cas 3
             cos_gamma3 = (L1**2 + L2**2 - D3**2) / (2 * L1 * L2)
             gamma3 = math.acos(np.clip(cos_gamma3, -1.0, 1.0))
             test_cases[2]["th2_exp"] = math.degrees(math.pi - gamma3)

             alpha3 = math.atan2(0.0, L1)
             cos_beta3 = (L1**2 + D3**2 - L2**2) / (2 * L1 * D3)
             beta3 = math.acos(np.clip(cos_beta3, -1.0, 1.0))
             test_cases[2]["th1_exp"] = math.degrees(alpha3 - beta3)
        else:
             test_cases[2]["ok_exp"] = False # Cas 3 n'est pas atteignable


    # Exécution et vérification des tests
    all_tests_passed = True
    for test in test_cases:
        target_x = test["x"]
        target_y = test["y"]
        ok_exp = test["ok_exp"]
        th1_exp_deg = test.get("th1_exp", None)
        th2_exp_deg = test.get("th2_exp", None)

        print(f"\n--- Test {test['id']}: Cible ({target_x:.4f}, {target_y:.4f}) ---")
        # ... [Reste du code de test identique à la version précédente] ...
        if ok_exp:
             print(f"  Attendu: Succès IK", end="")
             if th1_exp_deg is not None:
                 print(f" -> th1~{th1_exp_deg:.1f}°, th2~{th2_exp_deg:.1f}°")
             else:
                 print("")
        else:
             print(f"  Attendu: Échec IK")

        th1_rad, th2_rad, ok = calculate_ik(target_x, target_y)
        test_passed = True

        if ok:
            th1_deg = math.degrees(th1_rad)
            th2_deg = math.degrees(th2_rad)
            print(f"  Résultat: Succès IK -> th1={th1_deg:.1f}°, th2={th2_deg:.1f}°")

            fk_x, fk_y = calculate_fk_wrist(th1_deg, th2_deg)
            fk_ok = check_close(fk_x, target_x) and check_close(fk_y, target_y)
            print(f"  Vérif FK -> ({fk_x:.4f}, {fk_y:.4f}) -> {'OK' if fk_ok else 'ERREUR FK!'}")
            if not fk_ok: test_passed = False

            if ok_exp and th1_exp_deg is not None:
                 angles_ok = check_close(th1_deg, th1_exp_deg, tol=0.1) and check_close(th2_deg, th2_exp_deg, tol=0.1)
                 if not angles_ok:
                      print(f"  ATTENTION: Angles diffèrent des attendus ({th1_exp_deg:.1f}°, {th2_exp_deg:.1f}°)")
                      # On ne considère pas ça comme un échec critique si FK est ok, mais c'est bon à savoir
        else:
            print(f"  Résultat: Échec IK")

        if ok != ok_exp:
            print(f"  ERREUR: Le résultat succès/échec ({ok}) ne correspond pas à l'attendu ({ok_exp})!")
            test_passed = False

        print(f"--- Test {test['id']} {'PASSED' if test_passed else 'FAILED'} ---")
        if not test_passed: all_tests_passed = False
        print("-------------------------------------")

    print(f"\n======= SUMMARY: {'ALL TESTS PASSED' if all_tests_passed else 'SOME TESTS FAILED'} =======")