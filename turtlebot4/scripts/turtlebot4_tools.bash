#!/bin/bash

sudo apt update

# Installing Gazebo
sudo apt install -y ros-dev-tools
sudo apt install -y wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install -y ignition-fortress

# Installing turtlebot4 libs
sudo apt install -y ros-humble-turtlebot4-desktop
sudo apt install -y ros-humble-turtlebot4-simulator
