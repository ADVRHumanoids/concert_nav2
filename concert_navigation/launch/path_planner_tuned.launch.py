import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

# =============================================================================
# TUNED Nav2 bringup for the CONCERT omnidirectional robot.
#
# This is a NEW launcher that mirrors path_planner.launch.py but loads the
# tuned configuration set from  config/tuned/  and the tuned behavior tree
# config/tuned/behavior_tuned.xml.  The ORIGINAL path_planner.launch.py and the
# original config/*.yaml + behavior_tree/*.xml are left UNTOUCHED, so the old
# setup remains a working fallback.
#
# What changed (and why) is documented in NAV2_TUNING.md. Launch with:
#   ros2 launch concert_navigation path_planner_tuned.launch.py use_sim_time:=false
# =============================================================================

def generate_launch_description():
    use_sim_time_env = EnvironmentVariable(name='USE_SIM_TIME', default_value='false')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time_env,
        description='Use simulation time; keep false on the real robot unless /clock is active'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Paths to the TUNED configuration files (config/tuned/)
    package_dir = get_package_share_directory('concert_navigation')
    config_dir = os.path.join(package_dir, 'config', 'tuned')
    controller_yaml = os.path.join(config_dir, 'controller.yaml')
    bt_navigator_yaml = os.path.join(config_dir, 'bt_navigator.yaml')
    planner_yaml = os.path.join(config_dir, 'planner_server.yaml')
    recovery_yaml = os.path.join(config_dir, 'recovery.yaml')
    smoother_server_yaml = os.path.join(config_dir, 'smoother_server.yaml')
    global_costmap_yaml = os.path.join(config_dir, 'global_costmap.yaml')
    local_costmap_yaml = os.path.join(config_dir, 'local_costmap.yaml')
    velocity_smoother_yaml = os.path.join(config_dir, 'velocity_smoother.yaml')
    collision_monitor_yaml = os.path.join(config_dir, 'collision_monitor.yaml')
    behavior_tree_path = os.path.join(config_dir, 'behavior_tuned.xml')

    # Define remappings for tf topics
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Define the launch description with the essential nodes and plugins
    return LaunchDescription([
        use_sim_time_arg,

        # Controller Server with plugin and local costmap parameters
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml, {'use_sim_time': use_sim_time}, local_costmap_yaml],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]
        ),

        # Planner Server with plugin and global costmap parameters
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml, {'use_sim_time': use_sim_time}, global_costmap_yaml],
            remappings=remappings
        ),

        # Recovery Behavior Server with plugin
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[recovery_yaml, {'use_sim_time': use_sim_time}],
            output='screen',
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]
        ),

        # Behavior Tree Navigator (uses the tuned behavior tree)
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                bt_navigator_yaml,
                {'use_sim_time': use_sim_time},
                {'default_nav_to_pose_bt_xml': behavior_tree_path},
            ],
            remappings=remappings
        ),

        # Velocity Smoother (now actually in the command chain — collision_monitor
        # consumes cmd_vel_smoothed in the tuned collision_monitor.yaml)
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[velocity_smoother_yaml, {'use_sim_time': use_sim_time}],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]
        ),

        # Collision Monitor (final stage -> /omnisteering/cmd_vel)
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[collision_monitor_yaml, {'use_sim_time': use_sim_time}],
            remappings=remappings
        ),

        # Smoother Server kept DISABLED — SmacPlanner2D's internal smoother plus
        # MPPI's local re-optimization are sufficient (see NAV2_TUNING.md §6).
        # Node(
        #     package='nav2_smoother',
        #     executable='smoother_server',
        #     name='smoother_server',
        #     output='screen',
        #     parameters=[smoother_server_yaml, {'use_sim_time': use_sim_time}],
        #     remappings=remappings
        # ),

        # Lifecycle Manager for managing the state transitions of the navigation nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'use_sim_time': use_sim_time},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator',
                                        'velocity_smoother',
                                        'collision_monitor']}]
                                        # 'smoother_server']}]
        )
    ])
