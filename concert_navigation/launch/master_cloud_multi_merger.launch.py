import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory for 'ira_laser_tools'
    ira_laser_tools_share_dir = get_package_share_directory("ira_laser_tools")

    # Include the cloud_multi_merger.launch.py file
    cloud_multi_merger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ira_laser_tools_share_dir, 'launch', 'cloud_multi_merger.launch.py')
        )
    )

    return LaunchDescription([
        cloud_multi_merger_launch,
    ])
