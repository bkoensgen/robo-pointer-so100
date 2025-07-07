set -e

# Source the main ROS 2 installation.
source /opt/ros/humble/setup.bash

# Source the local workspace installation to make our packages available.
source /ros2_ws/install/setup.bash

# Execute the command passed to the container (e.g., from `docker-compose.yml`).
exec "$@"