#!/usr/bin/env bash
set -euo pipefail
sudo docker exec --user ubuntu -it ros2_humble_gui bash -lc 'source /opt/ros/humble/setup.bash; cd /home/ubuntu/projects/slam; [ -f install/setup.bash ] && source install/setup.bash; exec bash'
