#!/usr/bin/env bash
# Wrapper to run ROS 2 CLI commands in a clean environment to avoid PYTHONPATH/conda conflicts.
# Usage: scripts/ros2_cli.sh <ros2 subcommand...>

set -euo pipefail

if [ $# -eq 0 ]; then
  echo "Usage: $(basename "$0") <ros2 args...>" >&2
  exit 1
fi

CMD=("$@")

# Build a clean env, but propagate essential ROS variables only if set
ENV_ARGS=()
ENV_ARGS+=(HOME="$HOME")
ENV_ARGS+=(TERM="${TERM:-xterm-256color}")
ENV_ARGS+=(DISPLAY="${DISPLAY:-}")

# Only forward if non-empty to avoid issues like int("") on ROS_DOMAIN_ID
if [ -n "${ROS_DOMAIN_ID:-}" ]; then ENV_ARGS+=(ROS_DOMAIN_ID="$ROS_DOMAIN_ID"); fi
if [ -n "${ROS_LOCALHOST_ONLY:-}" ]; then ENV_ARGS+=(ROS_LOCALHOST_ONLY="$ROS_LOCALHOST_ONLY"); fi
if [ -n "${RMW_IMPLEMENTATION:-}" ]; then ENV_ARGS+=(RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION"); fi
if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then ENV_ARGS+=(FASTRTPS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"); fi
if [ -n "${CYCLONEDDS_URI:-}" ]; then ENV_ARGS+=(CYCLONEDDS_URI="$CYCLONEDDS_URI"); fi

env -i "${ENV_ARGS[@]}" \
  bash -lc '
    source /opt/ros/humble/setup.bash
    # Avoid user site interference
    export PYTHONNOUSERSITE=1
    exec ros2 "$@"
  ' bash "${CMD[@]}"
