from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=EnvironmentVariable(name='USE_SIM_TIME', default_value='false'),
        description='Use simulation time for LiDAR conversion and scan fusion'
    )
    pointcloud_topics_arg = DeclareLaunchArgument(
        'pointcloud_topics',
        default_value='/VLP16_lidar_back/velodyne_points /VLP16_lidar_front/velodyne_points',
        description='Whitespace-separated list of input PointCloud2 topics'
    )
    scan_frame_arg = DeclareLaunchArgument(
        'scan_frame',
        default_value='base_link',
        description='Frame used for the merged cloud and generated LaserScan'
    )
    cloud_destination_topic_arg = DeclareLaunchArgument(
        'cloud_destination_topic',
        default_value='/merged_cloud',
        description='Output topic for the merged point cloud'
    )
    scan_destination_topic_arg = DeclareLaunchArgument(
        'scan_destination_topic',
        default_value='/scan',
        description='Output topic for the generated laser scan'
    )
    min_height_arg = DeclareLaunchArgument(
        'min_height',
        default_value='-0.65',
        description='Minimum point height in scan_frame used for LaserScan projection'
    )
    max_height_arg = DeclareLaunchArgument(
        'max_height',
        default_value='0.60',
        description='Maximum point height in scan_frame used for LaserScan projection'
    )
    range_min_arg = DeclareLaunchArgument(
        'range_min',
        default_value='0.9',
        description='Minimum generated LaserScan range'
    )
    range_max_arg = DeclareLaunchArgument(
        'range_max',
        default_value='10.0',
        description='Maximum generated LaserScan range'
    )
    angle_min_arg = DeclareLaunchArgument(
        'angle_min',
        default_value='-3.1416',
        description='Minimum generated LaserScan angle'
    )
    angle_max_arg = DeclareLaunchArgument(
        'angle_max',
        default_value='3.1416',
        description='Maximum generated LaserScan angle'
    )
    angle_increment_arg = DeclareLaunchArgument(
        'angle_increment',
        default_value='0.007',
        description='Generated LaserScan angular resolution'
    )
    scan_time_arg = DeclareLaunchArgument(
        'scan_time',
        default_value='0.2',
        description='Generated LaserScan scan time'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    pointcloud_topics = LaunchConfiguration('pointcloud_topics')
    scan_frame = LaunchConfiguration('scan_frame')
    cloud_destination_topic = LaunchConfiguration('cloud_destination_topic')
    scan_destination_topic = LaunchConfiguration('scan_destination_topic')
    min_height = LaunchConfiguration('min_height')
    max_height = LaunchConfiguration('max_height')
    range_min = LaunchConfiguration('range_min')
    range_max = LaunchConfiguration('range_max')
    angle_min = LaunchConfiguration('angle_min')
    angle_max = LaunchConfiguration('angle_max')
    angle_increment = LaunchConfiguration('angle_increment')
    scan_time = LaunchConfiguration('scan_time')

    pointcloud_merger = Node(
        package='perception_utils_ros2',
        executable='pointcloud_merger',
        name='pointcloud_merger',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'destination_frame': scan_frame,
            'cloud_destination_topic': cloud_destination_topic,
            'pointcloud_topics': pointcloud_topics,
        }]
    )

    merged_cloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='merged_cloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', cloud_destination_topic),
            ('scan', scan_destination_topic),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': scan_frame,
            'transform_tolerance': 0.05,
            'min_height': ParameterValue(min_height, value_type=float),
            'max_height': ParameterValue(max_height, value_type=float),
            'angle_min': ParameterValue(angle_min, value_type=float),
            'angle_max': ParameterValue(angle_max, value_type=float),
            'angle_increment': ParameterValue(angle_increment, value_type=float),
            'scan_time': ParameterValue(scan_time, value_type=float),
            'range_min': ParameterValue(range_min, value_type=float),
            'range_max': ParameterValue(range_max, value_type=float),
            'use_inf': True,
            'inf_epsilon': 1.0,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        pointcloud_topics_arg,
        scan_frame_arg,
        cloud_destination_topic_arg,
        scan_destination_topic_arg,
        min_height_arg,
        max_height_arg,
        range_min_arg,
        range_max_arg,
        angle_min_arg,
        angle_max_arg,
        angle_increment_arg,
        scan_time_arg,
        pointcloud_merger,
        merged_cloud_to_scan,
    ])
