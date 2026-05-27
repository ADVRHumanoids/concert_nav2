import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ira_laser_tools_share_dir = get_package_share_directory("ira_laser_tools")

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=EnvironmentVariable(name='USE_SIM_TIME', default_value='false'),
        description='Use simulation time for cloud merger'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    cloud_multi_merger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ira_laser_tools_share_dir, 'launch', 'cloud_multi_merger.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        cloud_multi_merger_launch,
    ])
