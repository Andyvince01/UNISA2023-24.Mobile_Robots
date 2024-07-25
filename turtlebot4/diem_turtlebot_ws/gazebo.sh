#!/bin/bash

# Problems with dual Intel and Nvidia GPU systems

# If you are using a dual Intel/Nvidia system it could be the case that the simulator is being run under Intel instead of using the Nvidia GPU. Bugs can vary but there could problems with shadows, incorrect laser scans or other rendering related issues.
# sudo prime-select nvidia

# Build the packages with symlink installation
#colcon build --symlink-install

# Set up the environment to use installed packages
source install/setup.bash

# Launches Gazebo
ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py world:=square rviz:=true
