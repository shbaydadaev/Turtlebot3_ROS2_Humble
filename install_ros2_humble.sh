#!/bin/bash
# Apache License 2.0
# ROS 2 Humble Installation Script for Ubuntu 22.04
# Modified for ROS 2 by Shokhrukh Baydadaev

echo ""
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

echo "[Install prerequisites]"
sudo apt update && sudo apt install -y locales 
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale 
sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install /tmp/ros2-apt-source.deb

sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-${name_ros_version}-desktop

echo "[Environment setup and development tools]"
source /opt/ros/${name_ros_version}/setup.bash
sudo apt install -y python3-colcon-common-extensions \
                    python3-rosdep \
                    python3-vcstool \
                    git \
                    python3-pip \
                    nano \
                    net-tools \
                    x11-apps \
                    iputils-ping \
                    ros-${name_ros_version}-gazebo-ros-pkgs \
                    ros-${name_ros_version}-cartographer \
                    ros-${name_ros_version}-cartographer-ros \
                    ros-${name_ros_version}-navigation2 \
                    ros-${name_ros_version}-nav2-bringup

echo "[Initialize rosdep]"
sudo rosdep init || true
rosdep update

echo "[Create and build the colcon workspace]"
mkdir -p $HOME/$name_colcon_workspace/src
cd $HOME/$name_colcon_workspace/src

echo "📥 Cloning TurtleBot3 repositories..."
git clone -b ${name_ros_version} https://github.com/ROBOTIS-GIT/DynamixelSDK.git 
git clone -b ${name_ros_version} https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git 
git clone -b ${name_ros_version} https://github.com/ROBOTIS-GIT/turtlebot3.git 
git clone -b ${name_ros_version} https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git 
git clone -b ${name_ros_version} https://github.com/ROBOTIS-GIT/turtlebot3_applications.git 
git clone -b ${name_ros_version} https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs.git

cd $HOME/$name_colcon_workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "[Setup user environment]"
{
    echo "source /opt/ros/$name_ros_version/setup.bash"
    echo "source \$HOME/$name_colcon_workspace/install/setup.bash"
    echo "source /usr/share/gazebo/setup.sh"
    echo "alias nb='nano ~/.bashrc'"
    echo "alias sb='source ~/.bashrc'"
    echo "alias cw='cd ~/$name_colcon_workspace'"
    echo "alias cs='cd ~/$name_colcon_workspace/src'"
    echo "alias cb='cd ~/$name_colcon_workspace && colcon build'"
    echo "export TURTLEBOT3_MODEL=burger"
    echo "export ROS_DOMAIN_ID=30"
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
} >> ~/.bashrc
source ~/.bashrc

echo "[✅ ROS 2 Humble Installation Complete!]"
