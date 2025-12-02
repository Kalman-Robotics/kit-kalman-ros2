#!/bin/bash

# source ROS 2 environment
source /opt/ros/humble/setup.bash

sudo apt update
# update rosdep inside ros2 workspace
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# install required ROS 2 packages
sudo apt install -y \
    ros-humble-urdf \
    ros-humble-xacro

# build and install project
cd /ros2_ws/
colcon build
source ./install/setup.bash

echo -e "\e[32m----------SETUP COMPLETE----------\e[0m"