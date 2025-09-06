#!/usr/bin/env bash
# Mock end-to-end demo launcher for robo_pointer_visual
# - Starts the full pipeline with the mock interface
# - Verifies key topics are up and prints sample messages
# Usage:
#   scripts/mock_demo.sh [nano|medium|large|/path/to/weights.pt] [camera_device]
# Examples:
#   scripts/mock_demo.sh            # auto-pick yolov8n.pt and /dev/video0
#   scripts/mock_demo.sh nano /dev/video2
#   scripts/mock_demo.sh /home/benja/ros2_ws/yolov8m.pt /dev/video0

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"

# 1) Environment (ROS overlay + PYTHONPATH)
# shellcheck disable=SC1091
source "$REPO_ROOT/scripts/env.sh" >/dev/null 2>&1 || true

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[err] ros2 not found in PATH. Ensure ROS 2 Humble is installed and sourced." >&2
  exit 2
fi

# 2) Resolve weights and camera
WEIGHTS_SEL="${1:-auto}"
CAMERA_DEV="${2:-/dev/video0}"

pick_weights() {
  case "$1" in
    nano) echo "$REPO_ROOT/yolov8n.pt" ;;
    medium) echo "$REPO_ROOT/yolov8m.pt" ;;
    large) echo "$REPO_ROOT/yolov8l.pt" ;;
    auto|*)
      for f in "$REPO_ROOT/yolov8n.pt" "$REPO_ROOT/yolov8m.pt" "$REPO_ROOT/yolov8l.pt"; do
        [ -f "$f" ] && echo "$f" && return 0
      done
      # Fall back to the provided arg if it's a path
      [ -f "$1" ] && echo "$1" && return 0
      echo "" ;;
  esac
}

YOLO_WEIGHTS="$(pick_weights "$WEIGHTS_SEL")"
if [ -z "$YOLO_WEIGHTS" ] || [ ! -f "$YOLO_WEIGHTS" ]; then
  echo "[err] YOLO weights not found. Place yolov8n.pt in repo root or pass a path." >&2
  exit 3
fi

STAMP="$(date +%F_%H-%M-%S)"
LOGFILE="/tmp/mock_pipeline_${STAMP}.log"

echo "[info] Starting mock pipeline..."
echo "       weights = $YOLO_WEIGHTS"
echo "       camera  = $CAMERA_DEV"
echo "       log     = $LOGFILE"

# 3) Launch full pipeline (vision + controller + mock interface)
set +e
nohup ros2 launch robo_pointer_visual pipeline.launch.py \
  yolo_model:="$YOLO_WEIGHTS" \
  interface_type:=mock \
  camera_index:="$CAMERA_DEV" \
  device:=auto \
  publish_rate_hz:=15.0 \
  publish_static_tf:=true \
  tf_parent_frame:=Wrist_Pitch_Roll \
  tf_child_frame:=camera_frame \
  > "$LOGFILE" 2>&1 &
LAUNCH_PID=$!
set -e

cleanup() {
  echo
  echo "[info] Stopping mock pipeline (PID=$LAUNCH_PID)..."
  kill "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup INT TERM

# 4) Wait for key topics
echo "[info] Waiting for topics: /joint_states, /target_joint_angles, /image_debug"
for i in $(seq 1 60); do
  TL="$(ros2 topic list 2>/dev/null | tr '\n' ' ')"
  if echo "$TL" | grep -q "/joint_states" && \
     echo "$TL" | grep -q "/target_joint_angles" && \
     echo "$TL" | grep -q "/image_debug" ; then
    echo "[ok] Topics are up."
    break
  fi
  sleep 0.5
  if [ "$i" -eq 60 ]; then
    echo "[warn] Topics not detected in time. Current list:"; ros2 topic list || true
  fi
done

# 5) Optional: Load mock controller profile if available (non-fatal)
PROFILE_PATH="$(ros2 pkg prefix robo_pointer_visual 2>/dev/null)/share/robo_pointer_visual/config/demo_mock.yaml"
if [ -f "$PROFILE_PATH" ]; then
  ros2 param load /robot_controller_node "$PROFILE_PATH" >/dev/null 2>&1 || true
fi

# 6) Show one sample message from each key topic
echo "[info] Sampling topics (once each)..."
ros2 topic echo /detected_target_point --once --qos-durability volatile --qos-reliability reliable || true
ros2 topic echo /target_joint_angles --once --qos-durability volatile --qos-reliability reliable || true
ros2 topic echo /joint_states --once --qos-durability volatile --qos-reliability reliable || true

cat <<EOF
[next]
- Open rqt_image_view and select /image_debug to see detections.
- In RViz (optional), set Fixed Frame=Base and add TF + RobotModel.
- Adjust parameters live if needed, e.g.:
    ros2 param set /robot_controller_node dead_zone_px 12
    ros2 param set /robot_controller_node use_pid_control false
EOF

echo "[info] Mock pipeline running (PID=$LAUNCH_PID). Press Ctrl+C to stop."
wait "$LAUNCH_PID"
