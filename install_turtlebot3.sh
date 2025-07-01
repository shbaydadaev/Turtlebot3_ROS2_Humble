#!/bin/bash
# Apache License 2.0
# ROS 2 Humble Installation Script for Ubuntu 22.04
# Modified for ROS 2 by Shokhrukh Baydadaev

echo ""
echo "Install packages on Raspberry PI for TurtleBot3"
echo "[Note] Target OS version  >>> Ubuntu 22.04.x (Jammy Jellyfish)"
echo "[Note] Target ROS version >>> ROS 2 Humble Hawksbill"
echo "[Note] Colcon workspace   >>> \$HOME/turtlebot3_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read

# Set defaults
name_os_version=${name_os_version:="jammy"}
name_ros_version=${name_ros_version:="humble"}
name_colcon_workspace=${name_colcon_workspace:="turtlebot3_ws"}

# Prerequisite Setup
echo "[Install prerequisites]"
sudo apt update && sudo apt install -y locales 
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale 
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# ROS 2 Apt Source Setup
echo "[ROS 2 Apt Source Setup]"
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install /tmp/ros2-apt-source.deb

# ROS 2 & Package Installation
echo "[ROS 2 & Package Installation]"
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-${name_ros_version}-ros-base

# Development Tools & Dependencies
echo "[Environment setup and development tools]"
source /opt/ros/${name_ros_version}/setup.bash
sudo apt install -y python3-colcon-common-extensions \
                    python3-argcomplete \
                    libboost-system-dev \
                    build-essential \
                    python3-pip \
                    nano \
                    git \
                    python3-colcon-meson \
                    libudev-dev \
                    ros-${name_ros_version}-hls-lfcd-lds-driver \
                    ros-${name_ros_version}-turtlebot3-msgs \
                    ros-${name_ros_version}-dynamixel-sdk                 

# Initialize rosdep
echo "[Create and build the colcon workspace]"
sudo rosdep init
rosdep update
mkdir -p ~/$name_colcon_workspace/src
cd ~/$name_colcon_workspace/src

# Cloning TurtleBot3 repositories
echo "📥 Cloning TurtleBot3 repositories..."
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git 
git clone -b humble https://github.com/ROBOTIS-GIT/ld08_driver.git

# building the workspace
echo "[Build the colcon workspace]"
cd ~/$name_colcon_workspace/src/turtlebot3
rm -r turtlebot3_cartographer turtlebot3_navigation2
cd ~/turtlebot3_ws/
colcon build --symlink-install

# Setup rosdep
# ------------------------------

# Alias Section
echo "[Setup user environment]"
echo "alias nb='nano ~/.bashrc'" >> ~/.bashrc
echo "alias sb='source ~/.bashrc'" >> ~/.bashrc
echo "alias cw='cd ~/$name_colcon_workspace'" >> ~/.bashrc
echo "alias cs='cd ~/$name_colcon_workspace/src'" >> ~/.bashrc
echo "alias cb='cd ~/$name_colcon_workspace && colcon build'" >> ~/.bashrc

# Environment Setup
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc # Change Domain ID to avoid conflicts with other ROS 2 installations 
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
echo "source /opt/ros/$name_ros_version/setup.bash" >> ~/.bashrc
echo "source ~/$name_colcon_workspace/install/setup.bash" >> ~/.bashrc

# USB Port Settings for OpenCR
sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "[ROS source setup: <source ~/.bashrc>]"
echo "[✅ ROS 2 Humble Installation Complete!]"
