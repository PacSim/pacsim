#! /bin/bash
sudo apt update && 
rosdep update && 
rosdep install --from-paths src --ignore-src -y && 
colcon build &

wait $!

cd /home/ws/ &&
source ./install/setup.bash && 
ros2 launch pacsim example.launch.py
