import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Include the pointcloud_to_laserscan launch file
    pointcloud_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("concert_mapping"), 'launch','slam_toolbox_sync.launch.py'))
    )
    # Include the mux launch file
    mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("concert_mapping"), 'launch','map_saver.launch.py'))
    )

    return LaunchDescription([
        pointcloud_to_laserscan_launch,
        mux_launch
    ])
