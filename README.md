# Robo-Pointer Visuel (SO-ARM100 / LeRobot)

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

**Statut du Projet : 🛠️ Travail en Cours / Débogage Actif 🛠️**

Ce projet vise à contrôler un bras robotique SO-ARM100 (version LeRobot/Hugging Face) à l'aide d'une caméra montée sur le poignet ("eye-in-hand") pour détecter et suivre un objet de couleur rouge. Il s'agit d'un projet personnel d'apprentissage dans le cadre d'une reconversion vers la robotique logicielle.

**Objectif Principal :**
Implémenter un système de visuo-servage simple mais fonctionnel en utilisant ROS 2 Humble, Python et OpenCV.

**État Actuel (Branche `main` — Mai 2025) :**

* L'architecture ROS 2 (3 nœuds principaux) est en place et communique.
* Détection d’objet rouge basique via OpenCV (filtrage HSV) fonctionnelle.
* Interface bas niveau avec les moteurs Feetech (`scservo_sdk` + `lerobot`) établie, utilisant `GroupSyncRead`/`GroupSyncWrite`.
* Calibration manuelle des moteurs via un fichier JSON.
* Contrôle **Pan (horizontal)** via un contrôleur Proportionnel.
* **Cinématique Directe (FK)** et **Inverse (IK)** pour le plan vertical (épaule/coude) implémentée.
* **Contrôleur PI** pour l’axe Y cartésien implémenté.
* Compensation d’erreur angulaire locale (PI) pour le moteur de l’épaule (ID 2).
* **Défi Actuel :** tuning et stabilisation du contrôle vertical (moteur épaule ID 2 peinant à monter contre la gravité). Ajustement des gains PI local et exploration d’une compensation de gravité explicite.

---

## Table des Matières

* [Matériel Requis](#matériel-requis)
* [Logiciels Requis](#logiciels-requis)
* [Installation](#installation)

  * [1. Cloner le Dépôt](#1-cloner-le-dépôt)
  * [2. Environnement Conda](#2-environnement-conda)
  * [3. Installer LeRobot](#3-installer-lerobot)
  * [4. Workspace ROS 2](#4-workspace-ros-2)
  * [5. Calibration Manuelle (Crucial !)](#5-calibration-manuelle-crucial-)
  * [6. Règles Udev (Recommandé)](#6-règles-udev-recommandé)
  * [7. Permissions](#7-permissions)
* [Utilisation](#utilisation)
* [Structure du Projet](#structure-du-projet)
* [Prochaines Étapes](#prochaines-étapes)
* [Licence](#licence)
* [Contact](#contact)
* [Dépôt Matériel SO-ARM100](#dépôt-matériel-so-arm100)

---

## Matériel Requis

* Bras Robotique SO-ARM100 (version LeRobot/Hugging Face avec moteurs Feetech STS3215)
* Adaptateur U2D2 ou interface similaire pour les moteurs Feetech
* Caméra USB (testé avec Innomaker 1080P)
* Alimentation 5 V, \~5 A pour les moteurs
* Objet rouge vif pour la détection
* PC sous Ubuntu 22.04

## Logiciels Requis

* Ubuntu 22.04 LTS
* ROS 2 Humble Hawksbill (`ros-humble-desktop`)
* Miniconda ou Anaconda
* Git
* Tmux

## Installation

### 1. Cloner le Dépôt

```bash
# Adaptez ~/ros2_ws si votre workspace est ailleurs
git clone https://github.com/bkoensgen/robo-pointer-so100.git ~/ros2_ws/src/robo-pointer-so100
```

### 2. Environnement Conda

```bash
# Si vous n'avez pas déjà l'environnement de LeRobot
# conda create -n lerobot python=3.10
conda activate lerobot
```

### 3. Installer LeRobot

```bash
# Adaptez ~/lerobot si vous l'avez cloné ailleurs
git clone https://github.com/huggingface/lerobot.git ~/lerobot
cd ~/lerobot
pip install -e .
cd -  # Retour au dossier précédent
```

### 4. Workspace ROS 2

```bash
cd ~/ros2_ws
# Construire seulement ce package
colcon build --packages-select robo_pointer_visual
# Ou simplement `colcon build` pour tout reconstruire
```

Puis, ajoutez dans votre `~/.bashrc` si ce n’est pas déjà fait :

```bash
source ~/ros2_ws/install/setup.bash
```

### 5. Calibration Manuelle (Crucial !)

* Créez le fichier `~/.cache/lerobot/calibrations/so100/main_follower.json`
* Contient `motor_names`, `homing_offset`, `drive_mode`, `calib_mode`, etc.
* `homing_offset` = inverse des steps bruts mesurés au point zéro physique.
* `drive_mode` typique : `[0,1,0,0,1,0]` pour `[Pan,Lift,Elbow,WristFlex,WristRoll,Gripper]`.
* Inspirez-vous de la calibration automatique de `lerobot`, mais mesurez et corrigez manuellement.
* Créez les dossiers si nécessaire :

```bash
mkdir -p ~/.cache/lerobot/calibrations/so100
```

### 6. Règles Udev (Recommandé)

Créez `/etc/udev/rules.d/99-robot-devices.rules` :

```udev
# Feetech U2D2 Adapter
SUBSYSTEM=="tty", ATTRS{product}=="*U2D2*", ATTRS{serial}=="YOUR_U2D2_SERIAL", SYMLINK+="robot_arm", MODE="0666"

# USB Camera (Exemple générique)
SUBSYSTEM=="video4linux", KERNEL=="video[0-9]*", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY", ATTRS{serial}=="CAMERA_SERIAL", SYMLINK+="robot_camera", MODE="0666"
```

Rechargez les règles :

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 7. Permissions

Ajoutez votre utilisateur aux groupes :

```bash
sudo usermod -aG dialout $USER
sudo usermod -aG video  $USER
```

> **Important :** déconnectez-vous puis reconnectez-vous pour que les changements prennent effet.

## Utilisation

Rendez le script de lancement exécutable et lancez-le :

```bash
chmod +x ~/ros2_ws/start_robot.sh
cd ~/ros2_ws
./start_robot.sh
```

Le script crée une session `tmux` nommée `robot_dev` avec trois panneaux :

1. `vision_node`
2. `robot_controller_node`
3. `real_robot_interface` (logs en `~/real_robot_interface_test1.log`)

Commandes `tmux` utiles :

* Détacher : `Ctrl+b` puis `d`
* Ré-attacher : `tmux attach -t robot_dev`
* Tuer la session : `tmux kill-session -t robot_dev`

## Structure du Projet

Le dossier `~/ros2_ws/src/robo_pointer_visual/robo_pointer_visual/` contient :

* **vision\_node.py** : acquisition d’image, détection couleur, publication.
* **robot\_controller\_node.py** : calcul d’erreur visuelle, publication signal P.
* **real\_robot\_interface.py** : interface moteurs Feetech, calibration, FK/IK, contrôleurs PI, `GroupSyncWrite`.
* **kinematics.py** : fonctions FK/IK 2-DOF.
* **start\_robot.sh** : script de lancement.

## Prochaines Étapes

* Ajuster les gains PI locaux (épaule ID 2) pour stabiliser le contrôle vertical.
* Explorer une compensation de gravité explicite (URDF ou manuelle).
* Affiner les gains globaux pour un suivi fluide.
* Améliorer la détection visuelle.
* Nettoyer le code et la documentation.
* Ajouter des tests automatisés.
* Créer des démonstrations visuelles.

## Licence

Sous licence Apache 2.0. Voir le fichier `LICENSE`.

## Contact

Benjamin Koensgen – [b.koensgen@gmail.com](mailto:b.koensgen@gmail.com)
LinkedIn : linkedin.com/in/benjamin-koensgen

## Dépôt Matériel SO-ARM100

Pour la documentation et les fichiers CAD/URDF du SO-ARM100 (et SO-101), consultez également :
[https://github.com/TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
