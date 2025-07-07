# Stage 1: Base Image
FROM ros:humble-ros-core

# Stage 2: Environment Setup
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Stage 3: System & ROS Dependencies Installation
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-humble-xacro \
    libgl1-mesa-glx \
    udev \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Stage 3.5: Initialize rosdep
RUN rosdep init && \
    rosdep update

# Stage 4: Workspace Setup
WORKDIR /ros2_ws

# Stage 5: Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Stage 6: Copy ONLY assets and source code
COPY models/yolov8n.pt .
COPY src ./src

# Stage 7: Create scripts DIRECTLY inside the container to guarantee LF line endings

# Create entrypoint.sh
RUN <<EOF > /entrypoint.sh
#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
exec "\$@"
EOF

# Create start_robot.sh
RUN <<EOF > /ros2_ws/start_robot.sh
#!/bin/bash
SESSION_NAME="robot_dev"
CAMERA_DEVICE="/dev/robot_camera"
TARGET_CLASS="cup"
CONFIDENCE="0.6"
LOG_DIR="/ros2_ws/logs"
mkdir -p \$LOG_DIR
tmux kill-session -t \$SESSION_NAME 2>/dev/null || true
tmux new-session -d -s \$SESSION_NAME -n nodes
tmux split-window -h -t \$SESSION_NAME:nodes.0
tmux split-window -h -t \$SESSION_NAME:nodes.1
CMD_VISION="ros2 run robo_pointer_visual vision_node --ros-args -p camera_index:='\$CAMERA_DEVICE' -p target_class_name:='\$TARGET_CLASS' -p confidence_threshold:=\$CONFIDENCE"
tmux send-keys -t \$SESSION_NAME:nodes.0 "\$CMD_VISION" C-m
CMD_CONTROLLER="ros2 run robo_pointer_visual robot_controller_node"
tmux send-keys -t \$SESSION_NAME:nodes.1 "\$CMD_CONTROLLER" C-m
CMD_ROBOT="ros2 run robo_pointer_visual real_robot_interface --ros-args --log-level real_robot_interface:=debug > \$LOG_DIR/real_robot_interface.log 2>&1"
tmux send-keys -t \$SESSION_NAME:nodes.2 "\$CMD_ROBOT" C-m
tmux select-layout -t \$SESSION_NAME:nodes tiled
echo "Attaching to tmux session '\$SESSION_NAME'. To detach: Ctrl+b then d."
tmux attach-session -t \$SESSION_NAME
EOF

# Stage 8: Make the newly created scripts executable
RUN chmod +x /entrypoint.sh && chmod +x /ros2_ws/start_robot.sh

# Stage 9: Install ROS dependencies for the copied source code
RUN apt-get update && \
    . /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y

# Stage 10: Build the workspace
RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Stage 11: Final Configuration
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/ros2_ws/start_robot.sh"]