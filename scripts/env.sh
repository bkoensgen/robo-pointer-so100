#!/usr/bin/env bash
# Unified environment setup for running nodes and tests
# Note: avoid 'set -u' because ROS setup.bash may read unset vars (e.g., AMENT_TRACE_SETUP_FILES)
set -eo pipefail

# Resolve repo root (this file is in scripts/)
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"

# 1) ROS 2 overlay (system + local install)
# Some ROS setup scripts reference unset vars; temporarily disable nounset if active
_had_nounset=0
case $- in
  *u*) _had_nounset=1; set +u;;
esac
if [ -f "/opt/ros/humble/setup.bash" ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [ -f "$REPO_ROOT/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$REPO_ROOT/install/setup.bash"
fi
if [ "$_had_nounset" = "1" ]; then
  set -u
fi

# 2) Python search path
# - Always include the package source for tests and scripts
export PYTHONPATH="${PYTHONPATH:-}:$REPO_ROOT/src/robo_pointer_visual"

# - If running from a Conda env, add its site-packages so Ultralytics (etc.) are visible
if [ -n "${CONDA_PREFIX:-}" ]; then
  pyver=$(python3 - <<'PY'
import sys; print(f"{sys.version_info[0]}.{sys.version_info[1]}")
PY
)
  conda_site="$CONDA_PREFIX/lib/python${pyver}/site-packages"
  if [ -d "$conda_site" ]; then
    export PYTHONPATH="$PYTHONPATH:$conda_site"
  fi
fi

# - Optionally include a custom path (e.g., LEROBOT_PATH) if the user exports it before sourcing
if [ -n "${LEROBOT_PATH:-}" ] && [ -d "$LEROBOT_PATH" ]; then
  export PYTHONPATH="$PYTHONPATH:$LEROBOT_PATH"
fi

echo "[env] Repo=$REPO_ROOT"
echo "[env] PYTHONPATH=$PYTHONPATH"
command -v ros2 >/dev/null 2>&1 && echo "[env] ros2 available: $(ros2 --version 2>/dev/null || true)" || echo "[env] ros2 not found (pure tests only)"
