#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time   = LaunchConfiguration('use_sim_time', default='true')
    slam_mode      = LaunchConfiguration('slam', default='true')   # true -> SLAM, false -> AMCL+map
    map_yaml       = LaunchConfiguration('map', default='')        # set to enable map mode
    namespace      = LaunchConfiguration('namespace', default='')

    # --- Where the 41068 params live ---
    bringup_pkg_share = get_package_share_directory('41068_ignition_bringup')

    # Nav2 params from 41068 config (change filename if your repo differs)
    nav2_params_file = LaunchConfiguration(
        'nav2_params_file',
        default=os.path.join(bringup_pkg_share, 'config', 'nav2_params.yaml')
    )

    # SLAM params (use 41068’s if you have them; otherwise you can override via CLI)
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(bringup_pkg_share, 'config', 'slam_params.yaml')
    )

    # ---------------- Nav2 bringup ----------------
    # A) Navigation servers (no map required) for SLAM mode
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        condition=IfCondition(slam_mode),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': nav2_params_file,
            'namespace': namespace,
        }.items()
    )

    # B) Full bringup with AMCL + map_server for map mode
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        condition=UnlessCondition(slam_mode),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': nav2_params_file,
            'map': map_yaml,
            'namespace': namespace,
        }.items()
    )

    # ---------------- SLAM toolbox (only when slam:=true) ----------------
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(slam_mode)
    )

    # ---------------- Your AutoWanderDepth node ----------------
    auto = Node(
        package='ve_behaviours',
        executable='auto_wander_depth',   # expose via console_scripts in setup.py
        name='auto_wander_depth',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # You can override these from the CLI if needed
            'cmd_vel_topic': 'cmd_vel',
            'depth_topic': '/camera/depth/image',
            'linear_speed': 0.3,
            'angular_speed': 0.6,
            'min_range': 0.5,
            'wall_buffer': 0.7,
            'world_name': 'large_demo',
            'entity_name': 'husky',
        }],
    )

    return LaunchDescription([
        # Common args
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value=''),
        # Mode selection
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),
        # Files (default to 41068 configs; override if your filenames differ)
        DeclareLaunchArgument('nav2_params_file', default_value=os.path.join(bringup_pkg_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('slam_params_file', default_value=os.path.join(bringup_pkg_share, 'config', 'slam_params.yaml')),
        # Bringup
        nav2_navigation,
        nav2_bringup,
        slam,
        auto
    ])
