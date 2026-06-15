"""Merge the VLP16 3D point clouds into a single /merged_cloud.

Robust to one OR two lidars (`use_front_lidar` / `use_back_lidar`).

This replaces the previous version, which `IncludeLaunchDescription`-ed
`ira_laser_tools/launch/cloud_multi_merger.launch.py` -- a file that does not
exist (ira_laser_tools only ships ROS1 `.launch` files). The working cloud merger
is `perception_utils_ros2`'s `pointcloud_merger` node, which already skips empty
clouds, so it tolerates a single lidar by design; we just feed it the enabled
topics in a valid destination frame.

Simulation adaptations vs. the real-robot launch:
  - input topic `velodyne_points` -> `points` (Gazebo/ros_gz bridge topic);
  - destination_frame uses the simulated scanner frame `<scanner>`;
  - use_sim_time defaults to true.

Examples:
  ros2 launch concert_navigation master_cloud_multi_merger.launch.py
  ros2 launch concert_navigation master_cloud_multi_merger.launch.py use_front_lidar:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def launch_setup(context, *args, **kwargs):
    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    use_front = _as_bool(LaunchConfiguration('use_front_lidar').perform(context))
    use_back = _as_bool(LaunchConfiguration('use_back_lidar').perform(context))
    front_scanner = LaunchConfiguration('front_scanner').perform(context)
    back_scanner = LaunchConfiguration('back_scanner').perform(context)
    cloud_topic = LaunchConfiguration('cloud_topic').perform(context)

    if not use_front and not use_back:
        return [LogInfo(msg='[master_cloud_multi_merger] ERROR: both lidars '
                            'disabled. Enable at least one of '
                            'use_front_lidar / use_back_lidar.')]

    cloud_topics = []
    if use_back:
        cloud_topics.append('/{}/{}'.format(back_scanner, cloud_topic))
    if use_front:
        cloud_topics.append('/{}/{}'.format(front_scanner, cloud_topic))

    destination_frame = front_scanner if use_front else back_scanner

    merger = Node(
        package='perception_utils_ros2',
        executable='pointcloud_merger',
        name='pointcloud_merger',
        output='screen',
        parameters=[{
            'destination_frame': destination_frame,
            'cloud_destination_topic': '/merged_cloud',
            'pointcloud_topics': ' '.join(cloud_topics),
            'use_sim_time': use_sim_time,
        }],
    )

    return [
        merger,
        LogInfo(msg='[master_cloud_multi_merger] merging [{}] -> /merged_cloud '
                    'in frame {}'.format(', '.join(cloud_topics), destination_frame)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=EnvironmentVariable(name='USE_SIM_TIME', default_value='true'),
            description='Use simulation time for cloud merger'),
        DeclareLaunchArgument(
            'use_front_lidar', default_value='true',
            description='Enable the FRONT lidar cloud input'),
        DeclareLaunchArgument(
            'use_back_lidar', default_value='true',
            description='Enable the BACK lidar cloud input'),
        DeclareLaunchArgument(
            'front_scanner', default_value='VLP16_lidar_front',
            description='Front lidar namespace'),
        DeclareLaunchArgument(
            'back_scanner', default_value='VLP16_lidar_back',
            description='Back lidar namespace'),
        DeclareLaunchArgument(
            'cloud_topic', default_value='points',
            description='Per-lidar PointCloud2 topic name under each scanner namespace'),
        OpaqueFunction(function=launch_setup),
    ])
