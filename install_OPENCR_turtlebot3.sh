#!/bin/bash
# Apache License 2.0
# Raspberry Pi OpenCR Setup for TurtleBot3
# Modified for ROS 2 by Shokhrukh Baydadaev

echo ""
echo "🚀 Starting OpenCR Firmware Flashing for TurtleBot3 (burger model)"
echo "⚠️ This will overwrite your OpenCR firmware."
read -p "Press ENTER to continue or CTRL+C to cancel..."

# Add armhf architecture and install required library
cd ~
sudo dpkg --add-architecture armhf  
sudo apt-get update  
sudo apt-get install libc6:armhf  

# Set OpenCR environment variables
export OPENCR_PORT=/dev/ttyACM0  
export OPENCR_MODEL=burger
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
