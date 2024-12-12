import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory for 'concert_mapping'
    concert_mapping_share_dir = get_package_share_directory("concert_mapping")

    # Include the slam_toolbox_sync.launch.py file
    slam_toolbox_sync_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(concert_mapping_share_dir, 'launch', 'slam_toolbox_sync.launch.py')
        )
    )

    # Include the map_saver.launch.py file
    map_saver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(concert_mapping_share_dir, 'launch', 'map_saver.launch.py')
        )
    )

    return LaunchDescription([
        slam_toolbox_sync_launch,
        map_saver_launch,
    ])