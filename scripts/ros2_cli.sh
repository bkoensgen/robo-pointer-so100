#!/usr/bin/env bash
# Wrapper to run ROS 2 CLI commands in a clean environment to avoid PYTHONPATH/conda conflicts.
# Usage: scripts/ros2_cli.sh <ros2 subcommand...>

set -euo pipefail

if [ $# -eq 0 ]; then
  echo "Usage: $(basename "$0") <ros2 args...>" >&2
  exit 1
fi

CMD=("$@")

# Preserve minimal env needed for terminals/X11, reconstruct ROS env
env -i HOME="$HOME" TERM="${TERM:-xterm-256color}" DISPLAY="${DISPLAY:-}" \
  bash -lc '
    source /opt/ros/humble/setup.bash
    # Avoid user site interference
    export PYTHONNOUSERSITE=1
    exec ros2 "$@"
  ' bash "${CMD[@]}"

