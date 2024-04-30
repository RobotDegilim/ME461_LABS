#!/usr/bin/env bash

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

cd /home/robot_degilim_labs/mnt/labs_ws
colcon build --symlink-install 
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
source /usr/share/gazebo/setup.sh 
echo "export GAZEBO_MODEL_PATH=\${GAZEBO_MODEL_PATH}:/opt/ros/humble/share/humble_gazebo/models" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc