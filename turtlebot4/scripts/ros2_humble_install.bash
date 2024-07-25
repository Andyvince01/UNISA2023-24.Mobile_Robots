#!/bin/bash

sudo apt update

sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade -y

sudo apt install -y ros-humble-desktop

cd /home/$USER
STRING="source /opt/ros/humble/setup.bash"
NLINES=$(cat .bashrc | grep -c "$STRING")
if [ $NLINES -eq 0 ]; then
echo $STRING >> .bashrc
fi
