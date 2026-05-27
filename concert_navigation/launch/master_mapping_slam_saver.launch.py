import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory for 'concert_mapping'
    concert_mapping_share_dir = get_package_share_directory("concert_mapping")

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=EnvironmentVariable(name='USE_SIM_TIME', default_value='false'),
        description='Use simulation time for SLAM and map saver'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include the slam_toolbox_sync.launch.py file
    slam_toolbox_sync_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(concert_mapping_share_dir, 'launch', 'slam_toolbox_sync.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Include the map_saver.launch.py file
    map_saver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(concert_mapping_share_dir, 'launch', 'map_saver.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_toolbox_sync_launch,
        map_saver_launch,
    ])
