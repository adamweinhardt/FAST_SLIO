# Use a more complete ROS Noetic base image
FROM ros:noetic-ros-base

# Set the working directory to /root/semantickitti2bag inside the container
WORKDIR /root/semantickitti2bag2

# Copy everything from the current directory into the container
COPY ./ /root/semantickitti2bag2

# Install necessary system and ROS dependencies
RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    python3-rosdep \
    ros-noetic-sensor-msgs \
    ros-noetic-rosbag \
    ros-noetic-tf \
    ros-noetic-cv-bridge \
    libgl1-mesa-glx \
    libglib2.0-0 && \
    # Initialize rosdep if not already done
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update && \
    rosdep install --from-paths /opt/ros/noetic/share --ignore-src -r -y && \
    # Install Python dependencies with specific versions
    pip3 install numpy==1.21.0 pandas==1.3.0 opencv-python progressbar2 pykitti tqdm && \
    # Clean up to reduce image size
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Source ROS environment
CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && exec bash"]
