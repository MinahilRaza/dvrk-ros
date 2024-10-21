# Use the base image for ROS Humble Desktop Full
FROM osrf/ros:humble-desktop-full

# Update the package list and upgrade installed packages
RUN apt-get update && apt-get upgrade -y

# Install colcon and required dependencies
RUN apt-get install -y \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-pykdl \
    python3-pip

# Create a symbolic link to ensure that python refers to python3
RUN ln -sf /usr/bin/python3.10 /usr/bin/python


# Install additional dependencies for dVRK
RUN apt-get install -y \
    libxml2-dev \
    libraw1394-dev \
    libncurses5-dev \
    qtcreator \
    swig \
    sox \
    espeak \
    cmake-curses-gui \
    cmake-qt-gui \
    git \
    subversion \
    libcppunit-dev \
    libqt5xmlpatterns5-dev \
    libbluetooth-dev \
    python3-pyudev \
    gfortran-9 \
    libhidapi-dev \
    ros-humble-joint-state-publisher* \
    ros-humble-xacro

# Set up the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    mkdir -p ~/ros2_ws/src && \
    cd ~/ros2_ws/src && \
    vcs import --input https://raw.githubusercontent.com/jhu-saw/vcs/main/ros2-dvrk-2.3.0.vcs --recursive && \
    cd ~/ros2_ws && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Source ROS2 and workspace setup scripts
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# Set the entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
