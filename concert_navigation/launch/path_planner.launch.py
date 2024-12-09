import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Paths to the configuration files
    controller_yaml = os.path.join(get_package_share_directory('concert_navigation'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('concert_navigation'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('concert_navigation'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('concert_navigation'), 'config', 'recovery.yaml')
    smoother_server_yaml = os.path.join(get_package_share_directory('concert_navigation'), 'config', 'smoother_server.yaml')
    global_costmap_yaml = os.path.join(get_package_share_directory('concert_navigation'), 'config', 'global_costmap.yaml')
    local_costmap_yaml = os.path.join(get_package_share_directory('concert_navigation'), 'config', 'local_costmap.yaml')
    velocity_smoother_yaml = os.path.join(get_package_share_directory('concert_navigation'), 'config', 'velocity_smoother.yaml')
    collision_monitor_yaml = os.path.join(get_package_share_directory('concert_navigation'), 'config', 'collision_monitor.yaml')

    # Define remappings for tf topics
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Define the launch description with the essential nodes and plugins
    return LaunchDescription([
        # Controller Server with plugin and local costmap parameters
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml, local_costmap_yaml],
            remappings=remappings + [('/cmd_vel', '/omnisteering/cmd_vel')] 
        ),

        # Planner Server with plugin and global costmap parameters
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml, global_costmap_yaml],
            remappings=remappings
        ),

        # Recovery Behavior Server with plugin
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen',
            remappings=remappings + [('/cmd_vel', '/omnisteering/cmd_vel')]
        ),

        # Behavior Tree Navigator with plugin and both costmap parameters
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml, global_costmap_yaml, local_costmap_yaml],
            remappings=remappings
        ),

        # Velocity Smoother with its specific configuration
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[velocity_smoother_yaml],
            remappings=remappings + [('/cmd_vel', '/omnisteering/cmd_vel')]
        ),

        # Collision Monitor with its specific configuration
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[collision_monitor_yaml],
            remappings=remappings
        ),

        # Smoother Server for smoothing the global path
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[smoother_server_yaml],
            remappings=remappings
        ),

        # Lifecycle Manager for managing the state transitions of the navigation nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator',
                                        'velocity_smoother',
                                        'collision_monitor',
                                        'smoother_server']}]
        )
    ])
