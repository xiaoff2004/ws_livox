#!/bin/bash

# Mid360 仿真构建和启动脚本

cd /home/q/ws_livox
colcon build --packages-select mid360_simulation --symlink-install
source install/setup.bash
export GAZEBO_IP=127.0.0.1
ros2 launch mid360_simulation mid360_simulation.launch.py
