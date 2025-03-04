FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

# Setup ROS 2 package sources
RUN apt update && apt install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# Install dependencies including ROS 2
RUN apt update && apt install -y \
    ros-humble-desktop \
    ros-dev-tools \
    build-essential cmake \
    git python3-pip wget curl \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies
RUN apt-get update && apt-get install -y \
    git python3-pip wget curl \
    build-essential cmake \
    ros-humble-desktop \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Install PX4 dependencies
RUN useradd -m px4_user && \
    echo 'px4_user ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER px4_user
WORKDIR /home/px4_user

# Clone PX4 Autopilot
RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git

# Install ROS 2 PX4 messages and uXRCE-DDS agent
RUN mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src \
    && git clone https://github.com/PX4/px4_msgs.git \
    && git clone https://github.com/PX4/px4_ros_com.git

# Build ROS 2 workspace
WORKDIR /home/px4_user/ros2_ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build

# Set environment variables
ENV ROS_DOMAIN_ID=0
ENV PX4_HOME_LAT=37.7749
ENV PX4_HOME_LON=-122.4194
ENV PX4_HOME_ALT=10.0

# Run PX4 simulation with Gazebo
CMD ["/bin/bash"]