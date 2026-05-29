import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=EnvironmentVariable(name='USE_SIM_TIME', default_value='true'),
        description='Use simulation time for all included navigation bringup nodes'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 1. Declare a launch argument for map_nav with default 'true'
    map_nav_arg = DeclareLaunchArgument(
        name='map_nav',
        default_value='true',
        choices=['true', 'false'],
        description='If true, run SLAM + navigation simultaneously. If false, run only SLAM.'
    )

    # 2. Share directories
    odom_share_dir = get_package_share_directory('concert_odometry_ros2')
    navigation_share_dir = get_package_share_directory('concert_navigation')
    perception_share_dir = get_package_share_directory('perception_utils_ros2')

    # 3. Always launch odometry
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odom_share_dir, 'launch', 'concert_odometry.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. Always launch lidar / point cloud
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share_dir, 'launch', 'master_lidar_conversion_fuse.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 5. Always launch SLAM & map saver
    slam_and_saver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share_dir, 'launch', 'master_mapping_slam_saver.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 6. Launch path planner only if map_nav == true
    path_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share_dir, 'launch', 'path_planner.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('map_nav'))
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_nav_arg,
        odom_launch,
        lidar_launch,
        slam_and_saver_launch,
        path_planner_launch
    ])
