# Use official ROS Noetic base image (based on Ubuntu 20.04)
FROM ros:noetic-ros-core-focal

# Set environment variables for non-interactive installations
ENV DEBIAN_FRONTEND=noninteractive

# Update and install necessary tools, including RViz, rqt_bag, and eigen_conversions
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    libpcl-dev \
    libeigen3-dev \
    libceres-dev \
    ros-noetic-pcl-ros \
    ros-noetic-eigen-conversions \
    ros-noetic-rviz \
    ros-noetic-rqt-bag \
    ros-noetic-rqt-bag-plugins \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Set up ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]

# Create a workspace for FAST-LIO
RUN mkdir -p /fast_lio_ws/src
WORKDIR /fast_lio_ws

# Increase Git buffer size to handle large repository clones
RUN git config --global http.postBuffer 1048576000 \
    && git config --global http.lowSpeedLimit 0 \
    && git config --global http.lowSpeedTime 999999

# Delete the existing FAST_LIO directory before cloning to ensure fresh pull
RUN rm -rf /fast_lio_ws/src/FAST_LIO && \
    git clone https://github.com/adamweinhardt/FAST_SLIO.git /fast_lio_ws/src/FAST_LIO && \
    cd /fast_lio_ws/src/FAST_LIO && git submodule update --init

# Clone and build Livox ROS Driver
RUN mkdir -p /ws_livox/src && cd /ws_livox/src && \
    git clone https://github.com/Livox-SDK/livox_ros_driver.git && \
    cd /ws_livox && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source Livox ROS Driver
RUN echo "source /ws_livox/devel/setup.bash" >> ~/.bashrc

# Build the FAST-LIO package
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && source /ws_livox/devel/setup.bash && catkin_make"

# Source the FAST-LIO workspace
RUN echo "source /fast_lio_ws/devel/setup.bash" >> ~/.bashrc

RUN apt-get update && apt-get install -y libusb-1.0-0-dev

# Expose ports and set up display for RViz or any GUI application
RUN apt-get update && apt-get install -y x11-apps
ENV QT_X11_NO_MITSHM=1
RUN apt-get install -y mesa-utils
ENV DISPLAY=:1

# Allow access to host X server
RUN echo "export DISPLAY=$DISPLAY" >> ~/.bashrc
RUN echo "export QT_X11_NO_MITSHM=1" >> ~/.bashrc

# Volume for sharing data with host
VOLUME ["/home/docker_ws"]

# Set the default command to source the environment and open a bash shell
CMD ["bash"]
