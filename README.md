# TurtleBot3_ROS2_Humble (Editing...)

A quick-start guide and installation scripts for setting up [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) on **ROS 2 Humble Hawksbill** (Ubuntu 22.04).

---

## 🚀 Features

- Automated installation script for ROS 2 Humble and TurtleBot3 packages
- Ready-to-build colcon workspace setup
- Quick start aliases and environment variables

---

## ⚡ Requirements

- Make sure you’re running the latest Windows 11 with all updates (or Windows 10). (Open Settings → Windows Update → Check for updates)
- WSL2 (Windows Subsystem for Linux v2) >>> Open PowerShell as Administrator and run
- Install Ubuntu 22.04 from the Microsoft Store or with:

```powershell
wsl --install -d Ubuntu-22.04
```

Launch Ubuntu from the Start menu and set your Linux username/password.

- If you already have WSL, ensure it’s version 2:

```powershell
wsl --list --verbose
wsl --set-version <distro_name> 2
```

- Launch Ubuntu from the Start menu and set your Linux username/password.
- Internet connection
- Basic familiarity with terminal commands

---

## 🖥️ Installation (using Shell)

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

---

## 🐳 Installation (using Docker)

**1. Clone this repository:**

```bash
git clone https://github.com/shbaydadaev/Turtlebot3_ROS2_Humble.git
cd Turtlebot3_ROS2_Humble
```

**2. Build the Docker image:**

```bash
docker compose up --build 
```

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
- For simulation and Gazebo, make sure you have a working graphical environment (e.g. GWSL)

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
