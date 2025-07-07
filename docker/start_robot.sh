# Nom de la session Tmux
SESSION_NAME="robot_dev"

# --- User Configuration ---
# These parameters are now passed directly to the ros2 run commands.
CAMERA_DEVICE="/dev/robot_camera"
TARGET_CLASS="cup" # Change the target object here (e.g., "cell phone", "bottle")
CONFIDENCE="0.6"
LOG_DIR="/ros2_ws/logs" # Log directory inside the container

# --- Script Logic ---

# Ensure log directory exists
mkdir -p $LOG_DIR

# Kill any pre-existing session with the same name for a clean start.
tmux kill-session -t $SESSION_NAME 2>/dev/null || true

# Create a new, detached tmux session.
tmux new-session -d -s $SESSION_NAME -n nodes

# Configure window panes.
tmux split-window -h -t $SESSION_NAME:nodes.0
tmux split-window -h -t $SESSION_NAME:nodes.1

# --- Launch Commands ---
# The complex environment setup (`SETUP_CMDS`) is no longer needed.

# Pane 0: vision_node
CMD_VISION="ros2 run robo_pointer_visual vision_node --ros-args \
    -p camera_index:='$CAMERA_DEVICE' \
    -p target_class_name:='$TARGET_CLASS' \
    -p confidence_threshold:=$CONFIDENCE"
tmux send-keys -t $SESSION_NAME:nodes.0 "$CMD_VISION" C-m

# Pane 1: robot_controller_node
CMD_CONTROLLER="ros2 run robo_pointer_visual robot_controller_node"
tmux send-keys -t $SESSION_NAME:nodes.1 "$CMD_CONTROLLER" C-m

# Pane 2: real_robot_interface
CMD_ROBOT="ros2 run robo_pointer_visual real_robot_interface --ros-args \
    --log-level real_robot_interface:=debug > $LOG_DIR/real_robot_interface.log 2>&1"
tmux send-keys -t $SESSION_NAME:nodes.2 "$CMD_ROBOT" C-m

# --- Finalization ---

# Select a tiled layout for a clear view of all panes.
tmux select-layout -t $SESSION_NAME:nodes tiled

# Attach to the tmux session.
echo "Attaching to tmux session '$SESSION_NAME'. To detach: Ctrl+b then d."
tmux attach-session -t $SESSION_NAME