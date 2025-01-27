import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1) Pull the environment variable 'USE_SIM_TIME', default to 'true' if not set
    #    so that if you do not run `export USE_SIM_TIME=...`, it will use `true`.
    use_sim_time_env = EnvironmentVariable(name='USE_SIM_TIME', default_value='true')

    # 2) Use this environment variable as the default for the 'use_sim_time' LaunchConfiguration
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time_env,
        description='Use simulation time (true for simulation, false for real hardware)'
    )

    # 3) Create a LaunchConfiguration object to reference 'use_sim_time' in our Node parameters
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Paths to the configuration files
    package_dir = get_package_share_directory('concert_navigation')
    controller_yaml = os.path.join(package_dir, 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(package_dir, 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(package_dir, 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(package_dir, 'config', 'recovery.yaml')
    smoother_server_yaml = os.path.join(package_dir, 'config', 'smoother_server.yaml')
    global_costmap_yaml = os.path.join(package_dir, 'config', 'global_costmap.yaml')
    local_costmap_yaml = os.path.join(package_dir, 'config', 'local_costmap.yaml')
    velocity_smoother_yaml = os.path.join(package_dir, 'config', 'velocity_smoother.yaml')
    collision_monitor_yaml = os.path.join(package_dir, 'config', 'collision_monitor.yaml')
    behavior_tree_path = os.path.join(package_dir, 'behavior_tree', 'behavior.xml')

    # Define remappings for tf topics
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Nodes definitions with parameter overrides to respect the "use_sim_time" LaunchConfiguration
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        # The order below matters: later dictionary entries override previous ones
        parameters=[
            controller_yaml,             # Original YAML
            {'use_sim_time': use_sim_time},  # Override with launch argument
            local_costmap_yaml
        ],
        remappings=remappings + [('/cmd_vel', '/omnisteering/cmd_vel')]
    )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            planner_yaml,
            {'use_sim_time': use_sim_time},
            global_costmap_yaml
        ],
        remappings=remappings
    )

    recoveries_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='recoveries_server',
        output='screen',
        parameters=[
            recovery_yaml,
            {'use_sim_time': use_sim_time}
        ],
        remappings=remappings + [('/cmd_vel', '/omnisteering/cmd_vel')]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            bt_navigator_yaml,
            {'use_sim_time': use_sim_time},  # override
            {
                'default_nav_to_pose_bt_xml': behavior_tree_path,
                'default_nav_through_poses_bt_xml': behavior_tree_path,
            },
            global_costmap_yaml,
            local_costmap_yaml
        ],
        remappings=remappings
    )

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            velocity_smoother_yaml
        ],
        remappings=remappings + [('/cmd_vel', '/omnisteering/cmd_vel')]
    )

    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[
            collision_monitor_yaml
        ],
        remappings=remappings
    )

    smoother_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[
            smoother_server_yaml
        ],
        remappings=remappings
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': [
                'planner_server',
                'controller_server',
                'recoveries_server',
                'bt_navigator',
                'velocity_smoother',
                'collision_monitor',
                'smoother_server'
            ]}
        ]
    )

    # 4) Return a LaunchDescription including the newly-declared argument and the nodes
    return LaunchDescription([
        use_sim_time_arg,
        controller_node,
        planner_node,
        recoveries_node,
        bt_navigator_node,
        velocity_smoother_node,
        collision_monitor_node,
        smoother_node,
        lifecycle_manager_node
    ])

