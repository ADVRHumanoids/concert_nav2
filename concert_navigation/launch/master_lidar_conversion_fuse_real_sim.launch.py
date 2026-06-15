"""Sim LiDAR -> 2D /scan pipeline copied from the real-robot implementation.

Robust to running with one OR two VLP16 lidars. Select which lidars are present
with `use_front_lidar` / `use_back_lidar` (both default true).

Simulation adaptations vs. the real robot:
  - input topic `velodyne_points` -> `points` (Gazebo/ros_gz bridge topic);
  - target/destination frames use `<scanner>` instead of `<scanner>_base_link`;
  - use_sim_time defaults to true.

Why this matters: the ira_laser_tools `laserscan_multi_merger` blocks in its
constructor until EVERY topic listed in `laserscan_topics` is advertised, and
then only publishes the merged `/scan` once a fresh scan from EVERY subscribed
topic has arrived (see laserscan_multi_merger.cpp: topic parser `while`-loop and
the `totalClouds == clouds_modified.size()` gate). So if a dead lidar is left in
the list, `/scan` is never published and the whole Nav2 stack stalls. This launch
therefore configures the merger with ONLY the enabled scan topics.

Examples:
  # both lidars (default)
  ros2 launch concert_navigation master_lidar_conversion_fuse.launch.py
  # back lidar only (front broken / absent)
  ros2 launch concert_navigation master_lidar_conversion_fuse.launch.py use_front_lidar:=false
  # front lidar only
  ros2 launch concert_navigation master_lidar_conversion_fuse.launch.py use_back_lidar:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _p2l_node(name, scanner, min_height, max_height, scan_time, use_sim_time):
    """One pointcloud_to_laserscan branch for a single VLP16."""
    return Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name=name,
        remappings=[
            ('cloud_in', '/{}/points'.format(scanner)),
            ('scan', '/{}/scan'.format(scanner)),
        ],
        parameters=[{
            'target_frame': scanner,
            'transform_tolerance': 0.01,
            'min_height': min_height,
            'max_height': max_height,
            'angle_min': -3.1416,   # full 360 deg
            'angle_max': 3.1416,
            'angle_increment': 0.007,
            'scan_time': scan_time,
            'range_min': 0.7,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time,
        }],
    )


def launch_setup(context, *args, **kwargs):
    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    use_front = _as_bool(LaunchConfiguration('use_front_lidar').perform(context))
    use_back = _as_bool(LaunchConfiguration('use_back_lidar').perform(context))
    front_scanner = LaunchConfiguration('front_scanner').perform(context)
    back_scanner = LaunchConfiguration('back_scanner').perform(context)

    if not use_front and not use_back:
        return [LogInfo(msg='[master_lidar_conversion_fuse] ERROR: both lidars '
                            'disabled. Enable at least one of '
                            'use_front_lidar / use_back_lidar.')]

    nodes = []
    scan_topics = []

    # NOTE: front/back use different height bands on purpose (mounting geometry).
    if use_front:
        nodes.append(_p2l_node('pointcloud_to_laserscan_front', front_scanner,
                               min_height=-0.5, max_height=3.0, scan_time=0.2,
                               use_sim_time=use_sim_time))
        scan_topics.append('/{}/scan'.format(front_scanner))

    if use_back:
        nodes.append(_p2l_node('pointcloud_to_laserscan_back', back_scanner,
                               min_height=-3.0, max_height=0.5, scan_time=0.05,
                               use_sim_time=use_sim_time))
        scan_topics.append('/{}/scan'.format(back_scanner))

    # Merge into a frame that exists for an enabled simulated lidar.
    destination_frame = front_scanner if use_front else back_scanner

    # Merger configured with ONLY the enabled scan topics (see module docstring).
    nodes.append(Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laserscan_multi_merger',
        output='screen',
        parameters=[{
            'destination_frame': destination_frame,
            'cloud_destination_topic': '/merged_cloud',
            'scan_destination_topic': '/scan',
            'laserscan_topics': ' '.join(scan_topics),
            'angle_min': -3.1416,
            'angle_max': 3.1416,
            'angle_increment': 0.007,
            'scan_time': 0.2,
            'range_min': 0.9,
            'range_max': 130.0,
            'use_sim_time': use_sim_time,
        }],
    ))

    nodes.append(LogInfo(msg='[master_lidar_conversion_fuse] front={} back={} '
                             '-> /scan from [{}] in frame {}'.format(
                                 use_front, use_back,
                                 ', '.join(scan_topics), destination_frame)))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=EnvironmentVariable(name='USE_SIM_TIME', default_value='true'),
            description='Use simulation time for LiDAR conversion and scan fusion'),
        DeclareLaunchArgument(
            'use_front_lidar', default_value='true',
            description='Enable the FRONT VLP16 pointcloud->scan branch'),
        DeclareLaunchArgument(
            'use_back_lidar', default_value='true',
            description='Enable the BACK VLP16 pointcloud->scan branch'),
        DeclareLaunchArgument(
            'front_scanner', default_value='VLP16_lidar_front',
            description='Front lidar namespace (topics + simulated <ns> frame)'),
        DeclareLaunchArgument(
            'back_scanner', default_value='VLP16_lidar_back',
            description='Back lidar namespace (topics + simulated <ns> frame)'),
        OpaqueFunction(function=launch_setup),
    ])
