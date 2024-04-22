#! /bin/bash
sudo apt update && 
rosdep update && 
rosdep install --from-paths src --ignore-src -y && 
colcon build && 
source ./install/setup.bash && 
ros2 launch pacsim example.launch.py & 
cd /home/ws/ &&
source ./install/setup.bash && 
ros2 launch foxglove_bridge foxglove_bridge_launch.xml # Foxglove bridge in parallel