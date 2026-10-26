# Ubuntu 22.04 + ROS 2 Humble, light enough for an 8 GB Mac. Visualize with Foxglove (no GUI in the container).
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-pip python3-pytest \
      python3-numpy python3-scipy python3-pandas python3-matplotlib python3-opencv \
      ros-humble-foxglove-bridge ros-humble-tf2-ros ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
ENV ROS_DOMAIN_ID=0
# Source ROS and the workspace automatically in interactive shells
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ws/ros2_ws/install/setup.bash ] && source /ws/ros2_ws/install/setup.bash' >> /root/.bashrc
CMD ["bash"]
