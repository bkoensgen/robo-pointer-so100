#!/usr/bin/env bash
# Mock end-to-end demo launcher for robo_pointer_visual
# - Starts the full pipeline with the mock interface
# - Verifies key topics are up and prints sample messages
# - Optional: records a bag and customizes common parameters
#
# Usage:
#   scripts/mock_demo.sh [-w {nano|medium|large|/path/to/weights.pt}] \
#                        [-c CAMERA] [-t CLASS] [-d {auto|cpu|cuda}] \
#                        [-p PUB_HZ] [-r FPS] [-W WIDTH] [-H HEIGHT] \
#                        [-C CONF] [-B BAG_BASENAME] [-- no-static-tf]
# Examples:
#   scripts/mock_demo.sh                            # auto: yolov8n.pt, /dev/video0
#   scripts/mock_demo.sh -w nano -c /dev/video2     # nano weights, specific camera
#   scripts/mock_demo.sh -w ~/ros2_ws/yolov8m.pt -C 0.35 -t bottle

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"

usage() {
  cat <<USAGE
Usage: $(basename "$0") [options]
Options:
  -w PATH|nano|medium|large   YOLO weights (default: auto search in repo)
  -c CAMERA                   Camera index/path (default: /dev/video0)
  -t CLASS                    Target class name (default: bottle)
  -d DEVICE                   Device auto|cpu|cuda (default: auto)
  -p HZ                       Publish rate Hz (default: 15.0)
  -r FPS                      Camera FPS (default: 30.0)
  -W WIDTH                    Frame width (default: 640)
  -H HEIGHT                   Frame height (default: 480)
  -C CONF                     Confidence threshold [0,1] (default: 0.5)
  -B BASENAME                 Record rosbag to BASENAME (optional)
  -- no-static-tf             Do not publish static camera TF
  -h                          Show this help
USAGE
}

# 1) Environment (ROS overlay + PYTHONPATH)
# shellcheck disable=SC1091
source "$REPO_ROOT/scripts/env.sh" >/dev/null 2>&1 || true

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[err] ros2 not found in PATH. Ensure ROS 2 Humble is installed and sourced." >&2
  exit 2
fi

# 2) Parse CLI
WEIGHTS_SEL="auto"
CAMERA_DEV="/dev/video0"
TARGET_CLASS="bottle"
DEVICE_SEL="auto"
PUB_HZ="15.0"
FPS="30.0"
WIDTH="640"
HEIGHT="480"
CONF="0.5"
BAG_BASENAME=""
PUBLISH_STATIC_TF="true"

while (( "$#" )); do
  case "$1" in
    -w) WEIGHTS_SEL="${2:-}"; shift 2;;
    -c) CAMERA_DEV="${2:-}"; shift 2;;
    -t) TARGET_CLASS="${2:-}"; shift 2;;
    -d) DEVICE_SEL="${2:-}"; shift 2;;
    -p) PUB_HZ="${2:-}"; shift 2;;
    -r) FPS="${2:-}"; shift 2;;
    -W) WIDTH="${2:-}"; shift 2;;
    -H) HEIGHT="${2:-}"; shift 2;;
    -C) CONF="${2:-}"; shift 2;;
    -B) BAG_BASENAME="${2:-}"; shift 2;;
    --) shift; break;;
    --no-static-tf) PUBLISH_STATIC_TF="false"; shift;;
    -h|--help) usage; exit 0;;
    *)
      echo "[warn] Unknown option: $1" >&2
      usage; exit 1;;
  esac
done

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
echo "       weights  = $YOLO_WEIGHTS"
echo "       camera   = $CAMERA_DEV"
echo "       class    = $TARGET_CLASS"
echo "       device   = $DEVICE_SEL"
echo "       img      = ${WIDTH}x${HEIGHT}@${FPS}"
echo "       pub_hz   = $PUB_HZ"
echo "       conf     = $CONF"
echo "       staticTF = $PUBLISH_STATIC_TF"
echo "       log      = $LOGFILE"

# 3) Launch full pipeline (vision + controller + mock interface)
set +e
set -m  # enable job control to obtain process group id
nohup ros2 launch robo_pointer_visual pipeline.launch.py \
  yolo_model:="$YOLO_WEIGHTS" \
  interface_type:=mock \
  camera_index:="$CAMERA_DEV" \
  device:="$DEVICE_SEL" \
  publish_rate_hz:="$PUB_HZ" \
  confidence_threshold:="$CONF" \
  frame_rate:="$FPS" \
  frame_width:="$WIDTH" \
  frame_height:="$HEIGHT" \
  target_class_name:="$TARGET_CLASS" \
  publish_static_tf:="$PUBLISH_STATIC_TF" \
  tf_parent_frame:=Wrist_Pitch_Roll \
  tf_child_frame:=camera_frame \
  > "$LOGFILE" 2>&1 &
LAUNCH_PID=$!
LAUNCH_PGID="$(ps -o pgid= "$LAUNCH_PID" | tr -d '[:space:]')"
set -e

cleanup() {
  echo
  echo "[info] Stopping mock pipeline (PID=$LAUNCH_PID, PGID=$LAUNCH_PGID)..."
  # Try graceful stop first
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  sleep 0.5
  # Ensure process group is terminated
  kill -TERM -"$LAUNCH_PGID" 2>/dev/null || true
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

# 6) Optional bag record
if [ -n "$BAG_BASENAME" ]; then
  echo "[info] Recording rosbag: $BAG_BASENAME"
  ros2 bag record -O "$BAG_BASENAME" \
    /image_debug /detected_target_point /joint_states /target_joint_angles \
    >/dev/null 2>&1 &
  BAG_PID=$!
  trap 'cleanup; kill "$BAG_PID" 2>/dev/null || true' INT TERM
fi

# 7) Show one sample message from each key topic
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
