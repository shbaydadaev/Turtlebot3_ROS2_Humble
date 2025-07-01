# TurtleBot3_ROS2_Humble + Docker

A quick-start guide and installation scripts for setting up [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) on **ROS 2 Humble Hawksbill** (Ubuntu 22.04 + Docker image).

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

**3. Check docker process ID and copy it, then get the access to running docker container:**

```bash
docker ps 
docker exec -it [docker_id] bash
```

---

## 📦 Useful Aliases (added to .bashrc)

- **cw** : Go to workspace → `cd ~/turtlebot3_ws`
- **cs** : Go to src folder in workspace → `cd ~/turtlebot3_ws/src`
- **cb** : Build workspace → `colcon build --symlink-install`
- **nb** : Edit .bashrc → `nano ~/.bashrc`
- **sb** : Source .bashrc → `source ~/.bashrc`

---

## 🛠️ Additional Notes

- **TURTLEBOT3_MODEL=burger** (If you are using waffle, change it to **waffle** )
- **ROS_DOMAIN_ID=30** (make sure you are using the same domain as your robot)
- **RMW_IMPLEMENTATION=rmw_fastrtps_cpp** (use the same RMW)
- For simulation and Gazebo, make sure you have a working graphical environment (e.g. install **Xlaunch**)

---

## 📚 References

- [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
- [Docker Official Docs](https://docs.docker.com/)

---

## ✨ Author

**Shokhrukh Baydadaev**
Contributions and issues welcome!
