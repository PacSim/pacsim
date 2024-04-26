#! /bin/bash
echo "HOSTNAME OUTPUT:"
hostname -I

echo "\n"

sudo apt update && 
rosdep update && 
rosdep install --from-paths src --ignore-src -y && 
colcon build &

wait $!

cd /home/ws/ &&
source ./install/setup.bash && 
ros2 launch pacsim example.launch.py & 

cd /home/ws/ &&
source ./install/setup.bash && 
ros2 launch foxglove_bridge foxglove_bridge_launch.xml # Foxglove bridge in parallel
