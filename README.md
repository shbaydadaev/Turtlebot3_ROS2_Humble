# TurtleBot3_ROS2_Humble (Editing...)

A quick-start guide and installation scripts for setting up [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) on **ROS 2 Humble Hawksbill** (Ubuntu 22.04 or Virtual Machine).

---

## 🚀 Features

- Automated installation script for ROS 2 Humble and TurtleBot3 packages
- Ready-to-build colcon workspace setup
- Quick start aliases and environment variables

---

## ⚡ Requirements

- Make sure you’re running the latest Windows 11 with all updates. (Open Settings → Windows Update → Check for updates)
- 

## 🖥️ Installation ROS2 for PC

**1. 🔁 Clone this repository (install git!!!):**

```bash
sudo apt install git
git clone https://github.com/shbaydadaev/Turtlebot3_ROS2_Humble.git
cd Turtlebot3_ROS2_Humble
```

**2. Make the install script executable and run it:**

```bash
sudo chmod +x install_ros2_humble.sh
./install_ros2_humble.sh
```

**3. ✅ Installation complete! Please run:**

```bash
source ~/.bashrc
```

Now you're ready to start using TurtleBot3 with ROS 2!

---

## 📦 Useful Aliases (added to .bashrc)

- cw : Go to colcon workspace
- cs : Go to src folder in workspace
- cb : Build workspace
- nb : Edit .bashrc using nano editor
- sb : Source .bashrc

---

## 🛠️ Additional Notes

- **TURTLEBOT3_MODEL=burger** (If you are using waffle, change it to **waffle** )
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

```yaml

If you need **example Dockerfile** or specific environment/run command for real robots, multi-container setups, or simulation, just ask!
```
