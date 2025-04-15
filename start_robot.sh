#!/bin/bash

# Nom de la session Tmux
SESSION_NAME="robot_dev"

# --- Configuration Utilisateur ---
# Chemin vers le dossier LeRobot (À VÉRIFIER/ADAPTER SI NÉCESSAIRE)
LEROBOT_PATH="/home/benja/lerobot"
# Chemin vers le site-packages de l'environnement Conda (À VÉRIFIER/ADAPTER SI NÉCESSAIRE)
CONDA_SITE_PACKAGES="/home/benja/miniconda3/envs/lerobot/lib/python3.10/site-packages"
# --- Fin Configuration Utilisateur ---

# Vérifier si Tmux est installé
if ! command -v tmux &> /dev/null
then
    echo "Erreur : tmux n'est pas installé. Veuillez l'installer avec 'sudo apt install tmux'."
    exit 1
fi

# Tuer toute session tmux existante avec le même nom pour éviter les conflits
tmux kill-session -t $SESSION_NAME 2>/dev/null || true

# Commande de setup de base (Source ROS, activate Conda, export PYTHONPATH)
SETUP_CMDS="echo '--- Configuration Environnement ---' && \
source /opt/ros/humble/setup.bash && \
echo 'ROS Humble OK' && \
source ~/ros2_ws/install/setup.bash && \
echo 'Workspace ROS OK (avec cv_bridge local)' && \
conda activate lerobot && \
echo 'Conda lerobot OK' && \
export PYTHONPATH=\"\${PYTHONPATH}:${LEROBOT_PATH}:${CONDA_SITE_PACKAGES}\" && \
echo 'PYTHONPATH OK' && \
echo '--- Lancement Nœud ---'"

# Créer une nouvelle session tmux détachée avec une fenêtre nommée "nodes"
tmux new-session -d -s $SESSION_NAME -n nodes

# --- Configurer les Panneaux ---
# Diviser la fenêtre verticalement (crée panneau 1 à droite)
tmux split-window -h -t $SESSION_NAME:nodes.0
# Diviser le nouveau panneau verticalement (crée panneau 2 à droite du panneau 1)
tmux split-window -h -t $SESSION_NAME:nodes.1

# --- Envoyer les commandes à chaque panneau ---

# Panneau 0: vision_node (Utilise /dev/robot_camera)
tmux send-keys -t $SESSION_NAME:nodes.0 "$SETUP_CMDS && ros2 run robo_pointer_visual vision_node --ros-args -p camera_index:='/dev/robot_camera'" C-m

# Panneau 1: robot_controller_node (CORRIGÉ)
tmux send-keys -t $SESSION_NAME:nodes.1 "$SETUP_CMDS && ros2 run robo_pointer_visual robot_controller_node" C-m

# Panneau 2: real_robot_interface (CORRIGÉ - avec log DEBUG)
tmux send-keys -t $SESSION_NAME:nodes.2 "$SETUP_CMDS && ros2 run robo_pointer_visual real_robot_interface" C-m

# Optionnel: Sélectionner une disposition (ex: tiled pour essayer d'égaliser la taille)
tmux select-layout -t $SESSION_NAME:nodes tiled

# Donner un petit délai pour que les commandes commencent à s'exécuter
sleep 2

# Attacher à la session tmux pour voir les terminaux
echo "Attaching to tmux session '$SESSION_NAME'. Pour détacher: Ctrl+b puis d. Pour réattacher: tmux attach -t $SESSION_NAME"
tmux attach-session -t $SESSION_NAME
