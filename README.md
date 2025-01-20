# Launch Ethercat Mster to communicate with the board on the Real Robot
Terminal window #1
```bash
reset && ecat_master
```
# Launch xbot2 on the Real Robot
Terminal window #2
```bash
xbot2-core --hw ec_imp -V
```
# Concert GUI Real Robot on Vsion Board
Terminal window #3
```bash
ssh_vsion
```
```bash
cd ~/data/forest_ws/src/concert_config/config/xbot2_gui_server
```
```bash
xbot2_gui_server concert_gui_config.yaml
```
# Concert Robot Workspace

This repository provides the necessary setup, scripts, and commands to launch and operate the **Concert** robot in a simulated environment. The setup includes **Gazebo** simulation, **SLAM**, and **Navigation** in **ROS 2** (Robot Operating System 2). Below are detailed instructions for building the Docker image, launching the robot, enabling navigation, running SLAM, saving/loading maps, and following waypoints.

## Prerequisites

Ensure you have the following installed:

- **Docker and Docker Compose**:
    - To install, follow the appropriate instructions for [Docker](https://docs.docker.com/get-docker/) and [Docker Compose](https://docs.docker.com/compose/install/) on Ubuntu.

## Setup and Docker Build

1. **Build the Docker Image**:
    ```bash
    docker-compose build
    ```

2. **Run the Docker Container**:
    ```bash
    docker-compose up -d
    ```

## Launching Concert in Gazebo

Before launching the Concert robot model, set the required environment variable to specify the resource path for Gazebo:

```bash
export GZ_SIM_RESOURCE_PATH=/home/user/data/forest_ws/ros_src/concert_description/concert_gazebo/models
```

To launch the Concert robot model in the default Gazebo world, execute the following command:

```bash
ros2 launch concert_gazebo modular.launch.py velodyne:=true
```

This command launches the Concert robot in a simulated Gazebo environment with a Velodyne LiDAR sensor enabled.

To open **RViz** for visualizing sensor data, the robot's position, and the environment in 3D, use the following command:

```bash
rviz2 --ros-args -p use_sim_time:=true
```

## Setting Up Odometry

To start the odometry process, which helps the robot estimate its position and movement:

```bash
ros2 launch concert_odometry_ros2 concert_odometry.launch.py
```

## Converting and Fusing PointCloud Data

To convert PointCloud data to LaserScan and fuse multiple LaserScans into a single `/scan` topic, use:

```bash
ros2 launch concert_navigation master_lidar_conversion_fuse.launch.py
```

This script converts the PointCloud data from the 3D LiDAR into a 2D LaserScan format suitable for SLAM and navigation.

## Enabling SLAM (Simultaneous Localization and Mapping)

To start mapping the environment using the SLAM Toolbox and automatically save the map:

```bash
ros2 launch concert_navigation master_mapping_slam_saver.launch.py
```

### Saving the Map

Once you've completed mapping, save the generated map using the `nav2_map_server` package:

```bash
ros2 run nav2_map_server map_saver_cli -f maps/myworld
```

This will save the map in the `maps/` directory under the filename `myworld`. Adjust the path and file name as needed.

## Starting the Navigation Stack

After completing the map creation, start the navigation stack by initializing localization using AMCL (Adaptive Monte Carlo Localization) to position the robot within the known map:

```bash
ros2 launch concert_localization localization.launch.py
```

This command enables AMCL, allowing the robot to localize itself in the previously created map.

### Starting Path Planning with Nav2

To enable obstacle-avoiding path planning and navigate the Concert robot on the built map:

```bash
ros2 launch concert_navigation path_planner.launch.py
```

## Autonomous Navigation and Simultaneous Mapping

For autonomous navigation while building a map simultaneously, use the SLAM Toolbox with its localization feature and the navigation stack. This setup enables the robot to explore the environment, build the map, and navigate autonomously.

1. **Start SLAM Toolbox with Navigation Enabled**:
    ```bash
    ros2 launch concert_navigation master_mapping_slam_saver.launch.py
    ```

2. **Activate the Navigation Stack**:
    After launching SLAM Toolbox, ensure that the navigation stack is enabled to allow for obstacle-avoiding path planning. Use the following command:

    ```bash
    ros2 launch concert_navigation path_planner.launch.py
    ```

## 3D Mapping with RTAB-Map

For 3D mapping, Concert uses RTAB-Map, which allows real-time 3D mapping and localization. Since Concert has two 3D PointCloud sensors, you will also need to fuse their data.

1. **Fuse the 3D PointClouds**:
    To fuse the data from the two 3D PointCloud sensors, use the following command:
    ```bash
    ros2 launch concert_navigation master_cloud_multi_merger.launch.py
    ```
2. **Launch RTAB-Map for 3D Mapping**:
    Use the following command to start the RTAB-Map process:
    ```bash
    ros2 launch concert_mapping rtab_vlp16.launch.py
    ```
### Saving PointClouds as `.pcd` Files

To save the fused PointCloud data into `.pcd` files for later use:

```bash
ros2 launch concert_mapping pointcloud_to_pcd.launch.py
```

The resulting `.pcd` files will be stored in the specified directory `concert_mapping/pointclouds/`.

### Replacing the PointCloud with Saved `.pcd` Data

To load the saved `.pcd` file and use it as the PointCloud source:

```bash
ros2 launch concert_mapping pcd_to_pointcloud.launch.py
```

This replaces the live sensor data with the pre-saved PointCloud data, enabling operations using previously captured environments.
