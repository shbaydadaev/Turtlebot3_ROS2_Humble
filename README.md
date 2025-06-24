# TurtleBot3_ROS2_Humble

A quick-start guide and installation scripts for setting up [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) on **ROS 2 Humble Hawksbill** (Ubuntu 22.04).

---

## 🚀 Features

- Automated installation script for ROS 2 Humble and TurtleBot3 packages
- Ready-to-build colcon workspace setup
- Quick start aliases and environment variables

---

## 🖥️ Requirements

- Ubuntu 22.04 (Jammy Jellyfish)
- Internet connection
- Basic familiarity with terminal commands

---

## ⚡ Installation

**1. Clone this repository:**
```bash
git clone https://github.com/shbaydadaev/Turtlebot3_ROS2_Humble.git
cd Turtlebot3_ROS2_Humble
```

**2. Make the install script executable and run it:**
```bash
chmod +x install_ros2_humble.sh
./install_ros2_humble.sh
```

**3. Source your environment (or open a new terminal):**
```bash
source ~/.bashrc
```

## 📦 Useful Aliases (added to .bashrc)
- cw : Go to colcon workspace
- cs : Go to src folder in workspace
- cb : Build workspace
- eb : Edit .bashrc
- sb : Source .bashrc