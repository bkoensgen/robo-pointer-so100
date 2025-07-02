# Nom de la session Tmux
SESSION_NAME="robot_dev"

# --- Configuration Utilisateur ---
# Chemins à vérifier/adapter si nécessaire
LEROBOT_PATH="/home/benja/lerobot"
CONDA_SITE_PACKAGES="/home/benja/miniconda3/envs/lerobot/lib/python3.10/site-packages"

# Paramètres ROS
CAMERA_DEVICE="/dev/video2"
TARGET_CLASS="cup" # Changer ici l'objet à suivre (ex: "cell phone", "bottle")
CONFIDENCE="0.6"

# --- Fin Configuration Utilisateur ---

# Vérifier si Tmux est installé
if ! command -v tmux &> /dev/null
then
    echo "Erreur : tmux n'est pas installé. Veuillez l'installer avec 'sudo apt install tmux'."
    exit 1
fi

# Tuer toute session tmux existante avec le même nom pour un redémarrage propre
tmux kill-session -t $SESSION_NAME 2>/dev/null || true

# Commande de setup de base, partagée par tous les panneaux
SETUP_CMDS="source /opt/ros/humble/setup.bash && \
source ~/ros2_ws/install/setup.bash && \
conda activate lerobot && \
export PYTHONPATH=\"\${PYTHONPATH}:${LEROBOT_PATH}:${CONDA_SITE_PACKAGES}\" && \
echo '--- Environnement OK, lancement du nœud ---'"

# Créer une nouvelle session tmux détachée
tmux new-session -d -s $SESSION_NAME -n nodes

# --- Configurer les Panneaux ---
tmux split-window -h -t $SESSION_NAME:nodes.0
tmux split-window -h -t $SESSION_NAME:nodes.1

# --- Envoyer les commandes à chaque panneau ---

# Panneau 0: vision_node (avec les nouveaux paramètres YOLO)
CMD_VISION="$SETUP_CMDS && ros2 run robo_pointer_visual vision_node --ros-args \
    -p camera_index:='$CAMERA_DEVICE' \
    -p target_class_name:='$TARGET_CLASS' \
    -p confidence_threshold:=$CONFIDENCE"
tmux send-keys -t $SESSION_NAME:nodes.0 "$CMD_VISION" C-m

# Panneau 1: robot_controller_node
CMD_CONTROLLER="$SETUP_CMDS && ros2 run robo_pointer_visual robot_controller_node"
tmux send-keys -t $SESSION_NAME:nodes.1 "$CMD_CONTROLLER" C-m

# Panneau 2: real_robot_interface (AVEC LE LOG-LEVEL CORRIGÉ)
CMD_ROBOT="$SETUP_CMDS && ros2 run robo_pointer_visual real_robot_interface --ros-args \
    --log-level real_robot_interface:=debug \
    -p yw_increment_scale:=0.00025 > ~/real_robot_interface_test1.log 2>&1"
tmux send-keys -t $SESSION_NAME:nodes.2 "$CMD_ROBOT" C-m

# Sélectionner une disposition pour bien voir les trois panneaux
tmux select-layout -t $SESSION_NAME:nodes tiled

# Attacher à la session tmux
echo "Attaching to tmux session '$SESSION_NAME'. Pour détacher: Ctrl+b puis d."
tmux attach-session -t $SESSION_NAME
