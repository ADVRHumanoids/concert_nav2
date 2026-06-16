import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1) Pull the environment variable 'USE_SIM_TIME', default to 'false' if not set.
    #    Real robot runs should not wait for /clock unless explicitly requested.
    use_sim_time_env = EnvironmentVariable(name='USE_SIM_TIME', default_value='false')

    # 2) Use this environment variable as the default for the 'use_sim_time' LaunchConfiguration
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (true for simulation, false for real hardware)'
    )

    # 3) Create a LaunchConfiguration object to reference 'use_sim_time' in our Node parameters
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Paths to the configuration files
    package_dir = get_package_share_directory('concert_navigation')
    config_yaml = os.path.join(package_dir, 'config', 'navigation.yaml')

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
            config_yaml,             # Original YAML
            {'use_sim_time': use_sim_time},  # Override with launch argument
        ],
        remappings=remappings + [('cmd_vel', '/omnisteering/cmd_vel')]
    )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            config_yaml,
            {'use_sim_time': use_sim_time},
        ],
        # remappings=remappings
    )

    recoveries_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='recoveries_server',
        output='screen',
        parameters=[
            config_yaml,
            {'use_sim_time': use_sim_time}
        ],
        remappings=remappings + [('cmd_vel', '/omnisteering/cmd_vel')]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            config_yaml,
            {'use_sim_time': use_sim_time},  # override
        ],
        # remappings=remappings
    )

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            config_yaml,
            {'use_sim_time': use_sim_time}
        ],
        # remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]
    )

    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[
            config_yaml,
            {'use_sim_time': use_sim_time}
        ],
        # remappings=remappings
    )

    smoother_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[
            config_yaml,
            {'use_sim_time': use_sim_time}
        ],
        # remappings=remappings
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'autostart': True},
            {'use_sim_time': use_sim_time},
            {'node_names': [
                'planner_server',
                'map_server',
                'controller_server',
                'recoveries_server',
                'bt_navigator',
                'velocity_smoother',
                # 'collision_monitor',
                'smoother_server'
            ]}
        ],
        # arguments=['--ros-args', '--log-level', 'debug']
    )
    
    map_server_node = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    output='screen',
    parameters=[
        config_yaml,
        {'use_sim_time': use_sim_time},
    ],
)

    # 4) Return a LaunchDescription including the newly-declared argument and the nodes
    return LaunchDescription([
        use_sim_time_arg,
        controller_node,
        planner_node,
        map_server_node,
        recoveries_node,
        bt_navigator_node,
        velocity_smoother_node,
        # collision_monitor_node,
        smoother_node,
        lifecycle_manager_node
    ])