# Use the base ROS2 humble image
FROM osrf/ros:humble-desktop-full

# Set environment variable to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV COLCON_WS=/root/turtlebot3_ws

# Install required packages and clean up
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    git \
    nano \
    python3-pip \
    net-tools \
    x11-apps \
    iputils-ping \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup && \
    rm -rf /var/lib/apt/lists/*

# Create workspace and clone TurtleBot3 packages
WORKDIR ${COLCON_WS}/src
RUN git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
    git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
    git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3.git && \
    git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git && \
    git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3_applications.git && \
    git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs.git

# Build workspace
WORKDIR ${COLCON_WS}
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Setup environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc

# Start container in bash
CMD ["bash"]
