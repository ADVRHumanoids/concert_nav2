import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=EnvironmentVariable(name='USE_SIM_TIME', default_value='false'),
        description='Use simulation time for LiDAR conversion and scan fusion'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    pointcloud_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("pointcloud_to_laserscan"),
                'launch',
                'pointcloud_to_laserscan.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("perception_utils_ros2"),
                'launch',
                'laserscan_multi_merger.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        pointcloud_to_laserscan_launch,
        mux_launch
    ])
