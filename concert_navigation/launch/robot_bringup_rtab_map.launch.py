import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 1. Share directories
    odom_share_dir = get_package_share_directory('concert_odometry_ros2')
    perception_share_dir = get_package_share_directory('perception_utils_ros2')
    mapping_share_dir = get_package_share_directory('concert_mapping')

    # 2. Always launch odometry
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odom_share_dir, 'launch', 'concert_odometry.launch.py')
        )
    )
    # 3. Always launch lidar
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share_dir, 'launch', 'master_lidar_conversion_fuse.launch.py')
        )
    )

    # 4. Always launch point_cloud
    merge_pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share_dir, 'launch', 'cloud_multi_merger.launch.py')
        )
    )

    # 5. Always launch rtabmap
    rtab_vlp16_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mapping_share_dir, 'launch', 'rtab_vlp16.launch.py')
        )
    )

    return LaunchDescription([
        odom_launch,
        lidar_launch,
        merge_pointcloud_launch,
        rtab_vlp16_launch,
    ])
