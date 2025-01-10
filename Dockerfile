FROM hhcmhub/xbot2-noble-dev:latest

# Switch to root to install additional ROS packages
USER root

# Install additional ROS packages and utilities
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-nav2* \
    ros-$ROS_DISTRO-cartographer* \
    && apt-get clean

# Switch back to user
USER user
CMD ["bash"]
