#!/bin/bash

# --- Configuration Utilisateur ---

# Nom de la session Tmux
SESSION_NAME="robot_dev"

# Chemins essentiels pour l'environnement lerobot
LEROBOT_PATH="/home/benja/lerobot"
CONDA_BASE_PATH="/home/benja/miniconda3"
CONDA_SITE_PACKAGES="/home/benja/miniconda3/envs/lerobot/lib/python3.10/site-packages"

# Configuration de la Vision
CAMERA_DEVICE="/dev/video2"
TARGET_CLASS="banana"
CONFIDENCE="0.5"
FLIP_CODE="-1"

# Configuration du Contrôleur
SCALE_X="0.0004"
SCALE_Y="0.0004"

# --- Fin Configuration Utilisateur ---


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
    -p camera_index:='$CAMERA_DEVICE' \
    -p target_class_name:='$TARGET_CLASS' \
    -p confidence_threshold:=$CONFIDENCE \
    -p flip_code:=$FLIP_CODE"
tmux send-keys -t $SESSION_NAME:robot_pipeline.0 "$CMD_VISION" C-m

# Panneau 1 (en bas à gauche): robot_controller_node
CMD_CONTROLLER="$SETUP_CMDS && ros2 run robo_pointer_visual robot_controller_node --ros-args \
    -p pixel_to_cartesian_scale_x:=$SCALE_X \
    -p pixel_to_cartesian_scale_y:=$SCALE_Y"
tmux send-keys -t $SESSION_NAME:robot_pipeline.1 "$CMD_CONTROLLER" C-m

# Panneau 2 (à droite): real_robot_interface
LOG_FILE="~/real_robot_interface_$(date +%F_%H-%M-%S).log"
CMD_ROBOT="$SETUP_CMDS && ros2 run robo_pointer_visual real_robot_interface --ros-args \
    --log-level real_robot_interface:=debug > ~/real_robot_interface_test1.log 2>&1"
tmux send-keys -t $SESSION_NAME:nodes.2 "$CMD_ROBOT" C-m
    --log-level real_robot_interface:=info > $LOG_FILE 2>&1"
tmux send-keys -t $SESSION_NAME:robot_pipeline.2 "$CMD_ROBOT" C-m

tmux select-layout -t $SESSION_NAME:robot_pipeline 'tiled'

echo "Attaching to tmux session '$SESSION_NAME'. Pour détacher: Ctrl+b puis d."
sleep 1
tmux attach-session -t $SESSION_NAME
