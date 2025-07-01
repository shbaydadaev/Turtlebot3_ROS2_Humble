#!/bin/bash
# Apache License 2.0
# Raspberry Pi OpenCR Setup for TurtleBot3
# Modified for ROS 2 by Shokhrukh Baydadaev

echo ""
echo "Raspberry Pi OpenCR setup for TurtleBot3"

sudo dpkg --add-architecture armhf  
sudo apt-get update  
sudo apt-get install libc6:armhf  

export OPENCR_PORT=/dev/ttyACM0  
export OPENCR_MODEL=burger
if [ ! -e "$OPENCR_PORT" ]; then
  echo "❌ OpenCR device not found at $OPENCR_PORT. Make sure it is connected via USB."
  exit 1
fi
rm -rf ./opencr_update.tar.bz2

# Download the firmware and required loader, then extract the file to prepare for upload
echo "📦 Downloading OpenCR firmware..."
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2   
echo "📂 Extracting firmware..."
tar -xvf opencr_update.tar.bz2 

cd ./opencr_update  
echo "⚡ Flashing firmware..."
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr  

echo "[✅ OpenCR Setup Complete!]"
