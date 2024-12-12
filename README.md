## Launching Concert in Gazebo

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
ros2 launch concert_nav2 master_lidar_conversion_fuse.launch.py
```

This script converts the PointCloud data from the 3D LiDAR into a 2D LaserScan format suitable for SLAM and navigation.

## Enabling SLAM (Simultaneous Localization and Mapping)

To start mapping the environment using the SLAM Toolbox and automatically save the map:

```bash
ros2 launch concert_nav2 master_mapping_slam_saver.launch.py
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
ros2 launch concert_nav2 path_planner.launch.py
```

## Autonomous Navigation and Simultaneous Mapping

For autonomous navigation while building a map simultaneously, use the SLAM Toolbox with its localization feature and the navigation stack. This setup enables the robot to explore the environment, build the map, and navigate autonomously.

1. **Start SLAM Toolbox with Navigation Enabled**:
    ```bash
    ros2 launch concert_nav2 master_mapping_slam_saver.launch.py
    ```

2. **Activate the Navigation Stack**:
    After launching SLAM Toolbox, ensure that the navigation stack is enabled to allow for obstacle-avoiding path planning. Use the following command:

    ```bash
    ros2 launch concert_nav2 path_planner.launch.py
    ```
