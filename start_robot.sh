#!/bin/bash

# --- Configuration Utilisateur ---

# Nom de la session Tmux
SESSION_NAME="robot_dev"

# Chemins essentiels pour l'environnement
LEROBOT_PATH="/home/benja/lerobot"
CONDA_BASE_PATH="/home/benja/miniconda3"
CONDA_SITE_PACKAGES="${CONDA_BASE_PATH}/envs/lerobot/lib/python3.10/site-packages"

# Configuration de la Vision
CAMERA_DEVICE="/dev/camera_robot"
TARGET_CLASS="apple"
CONFIDENCE="0.3"
FLIP_CODE="-1"
FRAME_ACQUIRE="1"

# Configuration du Contrôleur
SCALE_X="0.0001"
SCALE_Y="0.0001"

# Configuration Caméra
FRAME_WIDTH="640"
FRAME_HEIGHT="480"
FRAME_RATE="30"
VIDEO_FOURCC="MJPG"

# Par défaut, on utilise le modèle "large"
YOLO_CHECKPOINT="yolov8m.pt"

# On regarde le premier argument passé au script
if [ "$1" == "nano" ]; then
  YOLO_CHECKPOINT="yolov8n.pt"
elif [ "$1" == "medium" ]; then
  YOLO_CHECKPOINT="yolov8m.pt"
elif [ "$1" == "large" ]; then
  YOLO_CHECKPOINT="yolov8l.pt"
elif [ -n "$1" ]; then

  echo "Erreur : modèle '$1' non reconnu. Options valides : nano, medium, large."
  exit 1
fi

echo "✅ Utilisation du modèle YOLO : $YOLO_CHECKPOINT"

# --- Logique du Script ---

if ! command -v tmux &> /dev/null
then
    echo "Erreur : tmux n'est pas installé."
    exit 1
fi

echo "Nettoyage d'une session tmux '$SESSION_NAME' existante..."
tmux kill-session -t $SESSION_NAME 2>/dev/null || true

# Commande de setup de base, partagée par tous les panneaux
SETUP_CMDS="source /opt/ros/humble/setup.bash && \
source ~/ros2_ws/install/setup.bash && \
source ${CONDA_BASE_PATH}/etc/profile.d/conda.sh && \
conda activate lerobot && \
export PYTHONPATH=\"\${PYTHONPATH}:${LEROBOT_PATH}:${CONDA_SITE_PACKAGES}\" && \
echo '--- Environnement ROS & Lerobot OK, lancement du nœud ---'"

echo "Création de la nouvelle session tmux '$SESSION_NAME'..."
tmux new-session -d -s $SESSION_NAME -n "robot_pipeline" -x "$(tput cols)" -y "$(tput lines)"
tmux split-window -h -t $SESSION_NAME:robot_pipeline.0
tmux split-window -v -t $SESSION_NAME:robot_pipeline.1

# --- Envoyer les commandes à chaque panneau ---

# Panneau 0 (en haut à gauche): vision_node
CMD_VISION="$SETUP_CMDS && ros2 run robo_pointer_visual vision_node --ros-args \
    -p yolo_model:='$YOLO_CHECKPOINT' \
    -p camera_index:='$CAMERA_DEVICE' \
    -p target_class_name:='$TARGET_CLASS' \
    -p persistence_frames_to_acquire:='$FRAME_ACQUIRE' \
    -p confidence_threshold:=$CONFIDENCE \
    -p flip_code:=$FLIP_CODE" \
    -p frame_width:=$FRAME_WIDTH \
    -p frame_height:=$FRAME_HEIGHT \
    -p frame_rate:=$FRAME_RATE \
    -p video_fourcc:=$VIDEO_FOURCC \
    -p publish_rate_hz:=15 \
    -p camera_backend:=v4l2 \

tmux send-keys -t $SESSION_NAME:robot_pipeline.0 "$CMD_VISION" C-m

# Panneau 1 (en bas à gauche): robot_controller_node
CMD_CONTROLLER="$SETUP_CMDS && ros2 run robo_pointer_visual robot_controller_node --ros-args \
    -p pixel_to_cartesian_scale_x:=$SCALE_X \
    -p pixel_to_cartesian_scale_y:=$SCALE_Y"
tmux send-keys -t $SESSION_NAME:robot_pipeline.1 "$CMD_CONTROLLER" C-m

# Panneau 2 (à droite): real_robot_interface
LOG_FILE="~/real_robot_interface_$(date +%F_%H-%M-%S).log"
CMD_ROBOT="$SETUP_CMDS && ros2 run robo_pointer_visual real_robot_interface --ros-args \
    --log-level real_robot_interface:=info > $LOG_FILE 2>&1"
tmux send-keys -t $SESSION_NAME:robot_pipeline.2 "$CMD_ROBOT" C-m

tmux select-layout -t $SESSION_NAME:robot_pipeline 'tiled'

echo "Attaching to tmux session '$SESSION_NAME'. Pour détacher: Ctrl+b puis d."
sleep 1
tmux attach-session -t $SESSION_NAME
