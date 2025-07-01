#!/bin/bash
# Apache License 2.0
# ROS 2 Raspberry Pi Camera Installation Script for TurtleBot3
# Target OS version  >>> Ubuntu 22.04.x (Jammy Jellyfish)
# Modified for ROS 2 by Shokhrukh Baydadaev

echo ""
echo "📸 Raspberry Pi Camera Installation Script for ROS 2 Humble"

sudo apt update
sudo apt upgrade -y
sudo apt install -y python-pip git python3-jinja2 python3-colcon-meson
sudo apt install -y libboost-dev
sudo apt install -y libgnutls28-dev openssl libtiff-dev pybind11-dev
sudo apt install -y qtbase5-dev libqt5core5a libqt5widgets qttools5-dev-tools 
sudo apt install -y meson cmake
sudo apt install -y python3-yaml python3-ply
sudo apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev

# Install ROS camera driver
sudo apt install ros-humble-camera-ros
# create workspace
mkdir -p ~/camera_ws/src
cd ~/camera_ws/src

# check out libcamera
sudo apt -y install python3-colcon-meson

echo "📥 Cloning libcamera repo..."
git clone https://github.com/raspberrypi/libcamera.git
# Option B: raspberrypi fork with support for newer camera modules
# git clone https://github.com/raspberrypi/libcamera.git
# check out this camera_ros repository
git clone https://github.com/christianrauch/camera_ros.git

cd ~/camera_ws/
rosdep install -y --from-paths src --ignore-src --rosdistro humble --skip-keys=libcamera
colcon build --event-handlers=console_direct+
# echo "⚙️ Building libcamera..."
# meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
# ninja -C build
# sudo ninja -C build install
# sudo ldconfig

echo ""
echo "✅ Libcamera Installation Complete!"
echo "To launch the camera node:"
echo "  ros2 launch turtlebot3_bringup camera.launch.py"
echo "To view the image:"
echo "  rqt_image_view"
echo ""