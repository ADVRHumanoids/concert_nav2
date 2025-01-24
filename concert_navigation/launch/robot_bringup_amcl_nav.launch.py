import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    default_map_file = os.path.join(
        get_package_share_directory('concert_mapping'),
        'maps',
        'map_test',
        'my_map.yaml'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_file,
        description='Default map file to load'
    )

    # Get share directories
    odom_share_dir = get_package_share_directory('concert_odometry_ros2')
    navigation_share_dir = get_package_share_directory('concert_navigation')
    localization_share_dir = get_package_share_directory('concert_localization')

    # Launch odometry
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odom_share_dir, 'launch', 'concert_odometry.launch.py')
        )
    )

    # Launch lidar
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share_dir, 'launch', 'master_lidar_conversion_fuse.launch.py')
        )
    )

    # AMCL for localization with the saved map
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_share_dir, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'map_file': LaunchConfiguration('map_file')}.items()
    )

    # Path planner for navigation
    path_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share_dir, 'launch', 'path_planner.launch.py')
        )
    )

    return LaunchDescription([
        map_file_arg,
        odom_launch,
        lidar_launch,
        amcl_launch,
        path_planner_launch
    ])
