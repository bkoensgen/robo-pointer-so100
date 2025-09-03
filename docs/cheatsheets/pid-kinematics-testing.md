# PID + Cinématique (2R) — Cheatsheet Tests (1‑page)

Objectif: lire/écrire des tests utiles sans plonger dans toutes les dérivations.

## PID — Invariants et Rappels
- Erreur: `e = SP − PV`.
- Rôles:
  - P: correction rapide ∝ erreur; +overshoot si trop fort.
  - I: supprime l’offset (erreur statique); risque de windup (saturation).
  - D: amortit via la pente (anticipation); amplifie le bruit ⇒ filtrer.
- Unités: `Ki` en 1/s (`Kp/Ti`), `Kd` en s (`Kp·Td`). Vérifier cohérence.
- Anti‑windup: limiter l’intégrateur quand la sortie sature (clamping/back‑calc).
- Dérivée: sur la mesure (PV) pour éviter le « derivative kick ».
- Échantillonnage: `Ts` ≥ 10× plus rapide que la dynamique dominante.
- Setpoint weighting: pondérer P/D sur les sauts de consigne (réduit les « kicks »).

### Signaux diagnostiques (mapping tests)
- Sortie qui diverge/sature dans le mauvais sens ⇒ signe/magnitude `Kp`.
- Oscillations soutenues ⇒ `Kp` trop grand, `Kd` insuffisant, `Ki` trop agressif.
- Offset constant ⇒ `Ki` trop faible/absent (ou saturation non détectée).
- Bruit en sortie ⇒ `D` trop élevé ou non filtré.

## Cinématique 2R — Invariants testables
- Portée: `D = √(x²+y²)` doit être dans `[|L1−L2|, L1+L2]` sinon IK échoue.
- FK (poignet):
  - `x = L1 cos θ1 + L2 cos(θ1+θ2)`
  - `y = L1 sin θ1 + L2 sin(θ1+θ2)`
- IK (branche coude‑haut ici): round‑trip `FK(θ1,θ2) → IK(x,y) ≈ (θ1,θ2)`.
- Gravité (simplifiée): `adj2 = k2·cos(θ1)`, `adj3 = k3·cos(θ1+θ2)`.
- Poignet horizontal: `θwrist = − (θlift + θelbow)`.
- Offset caméra avant: soustraire `(dx,dy) = (offset·cos φ, offset·sin φ)` avec `φ` orientation finale.

### Tols / Cas limites
- Tolérances numériques: `abs_tol ≈ 1e−6` (angles en rad → convertir en deg si besoin).
- Éviter singularités (0°, 180°) pour round‑trip stables; préférer angles intermédiaires.
- Clamping: vérifier respect des bornes physiques (pan/lift/elbow/wrist).

## ROS 2 — Patterns de tests
- Unitaires purs d’abord (cinématique, calibration): zéro I/O.
- Nœuds: mocker dépendances lourdes (YOLO, caméra, bus série), init/teardown `rclpy` par test.
- Topics/params: utiliser noms relatifs paramétrables; vérifier valeurs par défaut et remap.
- Unités messages: `JointState.position` en radians (pas degrés).
- Dispositif d’inférence: param `device=auto` ⇒ CUDA si dispo sinon CPU; `half()` seulement sur CUDA.

## Checklists rapides (pour écrire un test)
- Quel invariant je vérifie ? (portée IK, round‑trip, signe P/pan, clamp…)
- Qu’est‑ce qui varie ? (entrée minimale, limite, bruit)
- Déterministe ? (pas de temps/réseau; mocks; seed fixée)
- Seuils réalistes ? (`1e−6`, `±1°`, bornes mécaniques)
- Observation externe uniquement (pas l’implémentation interne)

## Exemples d’assertions utiles
- IK hors‑portée: `ok is False` si `D` en dehors de l’intervalle.
- Round‑trip: `deg(IK(FK(θ))) ≈ θ` (branche identique; `abs_tol` serré).
- Pan centré: cible au centre ⇒ `pan ≈ 0 rad`.
- Gravité: `cos(0)=1`, `cos(90°)=0`, `cos(180°)=−1` ⇒ valeurs clés.
- Offset caméra: `φ=0°` ⇒ `dx=+offset, dy≈0`; `φ=90°` ⇒ `dx≈0, dy=+offset`.

---
Astuce: commence par un test par invariant « coût/valeur élevé » (portée IK, round‑trip, clamp). Ajoute ensuite des cas limites ciblés (singularités, saturations) pour solidifier.
