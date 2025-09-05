#!/usr/bin/env bash
# Unified test runner: pure (no ROS) and ROS-dependent tests
set -euo pipefail

MODE="${1:-auto}"   # auto|pure|ros
shift || true        # pass any extra args to pytest

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"

# Source common environment (adds package to PYTHONPATH; sources ROS if present)
# shellcheck disable=SC1091
source "$REPO_ROOT/scripts/env.sh"

PYTEST_BIN="pytest"
if ! command -v "$PYTEST_BIN" >/dev/null 2>&1; then
  # Fallback to module invocation if pytest entrypoint is absent
  PYTEST_BIN="python3 -m pytest"
fi

PURE_FILES=(
  "$REPO_ROOT/src/robo_pointer_visual/test/test_kinematics.py"
  "$REPO_ROOT/src/robo_pointer_visual/test/test_kinematics_extra.py"
)

ROS_FILES=(
  "$REPO_ROOT/src/robo_pointer_visual/test/test_calibration_logic.py"
  "$REPO_ROOT/src/robo_pointer_visual/test/test_calibration_edges.py"
  "$REPO_ROOT/src/robo_pointer_visual/test/test_controller_safety.py"
  "$REPO_ROOT/src/robo_pointer_visual/test/test_launch_pipeline_structure.py"
  "$REPO_ROOT/src/robo_pointer_visual/test/test_mock_interface.py"
)

have_rclpy() {
  python3 - <<'PY'
try:
  import rclpy  # noqa: F401
  print('yes')
except Exception:
  print('no')
PY
}

run_pure() {
  echo "[tests] Running pure tests (kinematics)"
  export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  eval "$PYTEST_BIN" -q "${PURE_FILES[@]}" "$@"
}

run_ros() {
  echo "[tests] Running ROS-dependent tests"
  export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  eval "$PYTEST_BIN" -q "${ROS_FILES[@]}" "$@"
}

case "$MODE" in
  pure)
    run_pure "$@"
    ;;
  ros)
    if [ "$(have_rclpy)" != "yes" ]; then
      echo "[tests] rclpy not available in current environment. Source ROS Humble first." >&2
      exit 2
    fi
    run_ros "$@"
    ;;
  auto|*)
    run_pure "$@"
    if [ "$(have_rclpy)" = "yes" ]; then
      run_ros "$@"
    else
      echo "[tests] Skipping ROS-dependent tests (rclpy not found)." >&2
    fi
    ;;
esac
