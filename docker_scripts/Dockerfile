FROM osrf/ros:humble-desktop

# Container environment configuration
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    HOME=/root \
    ROS_DISTRO=humble

WORKDIR /root

# Configure apt to retry on transient network/mirror errors
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80retries

# Install essential build tools, Gazebo Harmonic repo, and ROS utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    lsb-release \
    gnupg \
    curl \
    wget \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    locales \
    git \
    && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    gz-harmonic \
    libgz-sim8-plugins \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Create ROS2 workspace structure
RUN mkdir -p /root/ros2_ws/src
COPY tmp_sources/ /root/ros2_ws/src/
RUN find /root/ros2_ws/src -name joy_to_cmdvel.py -exec chmod +x {} \;
WORKDIR /root/ros2_ws

# Install ROS2 dependencies for Armando simulation
RUN apt-get update -o Acquire::Retries=5 && apt-get install -y --no-install-recommends \
    ros-humble-rclcpp \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-control-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-srvs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-urdf \
    ros-humble-urdf-launch \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-interfaces \
    ros-humble-gz-ros2-control \
    ros-humble-controller-manager \
    liborocos-kdl-dev \
    ros-humble-kdl-parser \
    ros-humble-geometry-msgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-plugins \
    ros-humble-moveit-kinematics \
    ros-humble-aruco-ros \
    ros-humble-aruco-msgs \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Build workspace
RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd /root/ros2_ws \
    && colcon build"

# Create a script that configures the ROS2 environment
RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /entrypoint.sh && \
    echo 'source /root/ros2_ws/install/local_setup.bash' >> /entrypoint.sh && \
    echo 'export GZ_VERSION=harmonic' >> /entrypoint.sh && \
    echo 'export GZ_SIM_RESOURCE_PATH=/root/ros2_ws/install/armando_description/share:$GZ_SIM_RESOURCE_PATH' >> /entrypoint.sh && \
    echo 'export ROS_DOMAIN_ID=97' >> /entrypoint.sh && \
    echo 'exec "$@"' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

# Configure the ROS2 environment for interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc \
    && echo "export GZ_VERSION=harmonic" >> /root/.bashrc \
    && echo "export GZ_SIM_RESOURCE_PATH=/root/ros2_ws/install/armando_description/share:\$GZ_SIM_RESOURCE_PATH" >> /root/.bashrc \
    && echo "export ROS_DOMAIN_ID=97" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
WORKDIR /root/ros2_ws
CMD ["/bin/bash", "-i"]