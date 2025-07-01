#!/bin/bash
# Apache License 2.0
# ROS 2 Raspberry Pi Camera Installation Script for TurtleBot3
# Target OS version  >>> Ubuntu 22.04.x (Jammy Jellyfish)
# Modified for ROS 2 by Shokhrukh Baydadaev

echo ""
echo "📸 Raspberry Pi Camera Installation Script for ROS 2 Humble"

sudo apt update
sudo apt upgrade -y
sudo apt install -y python3-pip git python3-jinja2 \
                    libboost-dev libgnutls28-dev openssl libtiff-dev pybind11-dev \
                    qtbase5-dev libqt5core5a libqt5widgets5 meson cmake \
                    python3-yaml python3-ply \
                    libglib2.0-dev libgstreamer-plugins-base1.0-dev

# Install ROS camera driver
sudo apt install ros-humble-camera-ros

# Build and Install libcamera
echo "📥 Cloning libcamera repo..."
cd ~ && git clone -b humble https://github.com/raspberrypi/libcamera.git
cd libcamera

echo "⚙️ Building libcamera..."
meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
ninja -C build
sudo ninja -C build install
sudo ldconfig

echo ""
echo "✅ Libcamera Installation Complete!"
echo "To launch the camera node:"
echo "  ros2 launch turtlebot3_bringup camera.launch.py"
echo "To view the image:"
echo "  rqt_image_view"
echo ""