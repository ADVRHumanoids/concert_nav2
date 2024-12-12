FROM hhcmhub/xbot2-noble-dev:latest

# Install additional ROS packages and utilities
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-nav2* \
    ros-$ROS_DISTRO-cartographer \
    && sudo apt-get clean

# Default command to run when starting the container
CMD ["bash"]