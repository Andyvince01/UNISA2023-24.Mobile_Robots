sudo apt install -y ntpdate

# Synchronize clock
sudo ntpdate ntp.ubuntu.com

sudo ./configure_discovery.sh

source ~/.bashrc

ros2 daemon stop; ros2 daemon start

# ros2 topic list
