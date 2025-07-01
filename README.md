# TurtleBot3_ROS2_Humble (Editing...)

A quick-start guide and installation scripts for setting up [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) on **ROS 2 Humble Hawksbill** (Ubuntu 22.04 or Virtual Machine).

---

## 🚀 Features

- Automated installation script for ROS 2 Humble and TurtleBot3 packages
- Ready-to-build colcon workspace setup
- Quick start aliases and environment variables

---

## ⚡ Requirements

**Before starting, make sure your system meets the following requirements:**

- Ubuntu 22.04 (Desktop version recommended)
- Works on physical machine or Virtual Machine (e.g., VirtualBox or VMware)
- At least 8 GB RAM (16 GB recommended for better performance) and at least 50 GB of free disk space
- Internet connection
- Git (Install using: **sudo apt install git**)

---

## 🖥️ Installation ROS2 for PC

**1. 🔁 Clone this repository (install git!!!):**

```bash
sudo apt install git
git clone https://github.com/shbaydadaev/Turtlebot3_ROS2_Humble.git
cd Turtlebot3_ROS2_Humble
```

**2. ⚙️ Make the install script executable and run it:**

```bash
sudo chmod +x install_ros2_humble.sh
./install_ros2_humble.sh
```

**3. ✅ Installation complete! Please run:**

```bash
source ~/.bashrc
```

---

## 🤖 Installation: ROS 2 on TurtleBot3 (Raspberry Pi)

**1. 🔁 Clone This Repository (Make Sure Git Is Installed):**

```bash
sudo apt update
sudo apt install git -y
sudo apt upgrade
git clone https://github.com/shbaydadaev/Turtlebot3_ROS2_Humble.git
cd Turtlebot3_ROS2_Humble
```

💡 **Note:** On the TurtleBot3, you should use `install_turtlebot3.sh` — not `install_ros2_humble.sh`.

**2. ⚙️ Make the install script executable and run it:**

```bash
sudo chmod +x install_turtlebot3.sh
./install_turtlebot3.sh
```

This script will install ROS 2 Humble and TurtleBot3 dependencies on the Raspberry Pi, along with environment setup.

**3. ✅ Installation complete! Please run:**

After installation, apply your environment settings:

```bash
source ~/.bashrc
```

**4. ⚙️ OpenCR Setup (Firmware Flashing):**

```bash
sudo chmod +x install_OPENCR_turtlebot3.sh
./install_OPENCR_turtlebot3.sh
```

This step flashes the OpenCR board with the correct firmware for ROS 2. Ensure your OpenCR is connected via USB.

**5. (Optional) 📷 Installation Camera Node:**

If your TurtleBot3 is equipped with a camera and you'd like to verify it's working, run

```bash
sudo chmod +x install_camera_turtlebot3.sh
./install_camera_turtlebot3.sh
```

To verify the camera:

```bash
ros2 launch turtlebot3_bringup camera.launch.py
rqt_image_view
```

🔁 **Reminder:** After running each install script (OpenCR, camera), run:

```bash
source ~/.bashrc
```

You’re now ready to use TurtleBot3 with ROS 2!

---

## 📦 Useful Aliases (added to .bashrc)

- **cw** : Go to workspace → `cd ~/turtlebot3_ws`
- **cs** : Go to src folder in workspace → `cd ~/turtlebot3_ws/src`
- **cb** : Build workspace → `colcon build --symlink-install`
- **nb** : Edit .bashrc → `nano ~/.bashrc`
- **sb** : Source .bashrc → `source ~/.bashrc`

---

## 🛠️ Additional Notes

- **TURTLEBOT3_MODEL=burger** (If you are using waffle, change it to `waffle`)
- **ROS_DOMAIN_ID=30** (make sure you are using the same domain as your robot)
- **RMW_IMPLEMENTATION=rmw_fastrtps_cpp** (use the same RMW)
- For simulation and Gazebo, make sure you have a working graphical environment (e.g. install **GWSL** from Microsoft store)

---

## 📚 References

- [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
- [Docker Official Docs](https://docs.docker.com/)

---

## ✨ Author

**Shokhrukh Baydadaev**
Contributions and issues welcome!
