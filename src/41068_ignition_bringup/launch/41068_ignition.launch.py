#!/usr/bin/env python3
# ===============================================================
# VerdantEye: Husky Ignition Launch (Drone-Style Layout)
# - Loads Husky robot in Ignition
# - Starts robot_state_publisher + EKF + Gazebo bridge
# - Optional: Nav2 + RViz
# - Launches bamboo randomizer, UI panels, and auto_wander_depth
# ===============================================================

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    ld = LaunchDescription()

    # === Paths ===
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])
    models_path = PathJoinSubstitution([pkg_path, 'models'])
    worlds_path = PathJoinSubstitution([pkg_path, 'worlds'])

    # === Env: ensure Ignition can find model:// URIs ===
    for env_var in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH', 'GAZEBO_MODEL_PATH'):
        ld.add_action(SetEnvironmentVariable(
            name=env_var,
            value=[models_path, os.pathsep, os.environ.get(env_var, '')]
        ))

    # === Launch Arguments ===
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    nav2 = LaunchConfiguration('nav2')
    world = LaunchConfiguration('world')
    ui_flag = LaunchConfiguration('ui')
    bamboo_seed = LaunchConfiguration('bamboo_seed')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation time'))
    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='False', description='Launch RViz2'))
    ld.add_action(DeclareLaunchArgument(
        'nav2', default_value='True', description='Launch Nav2'))
    ld.add_action(DeclareLaunchArgument(
        'world', default_value='large_demo',
        choices=['simple_trees', 'large_demo'],
        description='Which world to load')
    )
    ld.add_action(DeclareLaunchArgument(
        'ui', default_value='False', description='Launch VerdantEye UI bundle'))
    ld.add_action(DeclareLaunchArgument(
        'bamboo_seed', default_value='0', description='Seed for bamboo randomizer (0 = time-based)'))

    # === Robot Description (URDF/Xacro) ===
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf', 'husky.urdf.xacro'])]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(robot_state_publisher_node)

    # === EKF Localization ===
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, 'robot_localization.yaml']),
            {'use_sim_time': use_sim_time}
        ]
    )
    ld.add_action(ekf_node)

    # === Gazebo (Ignition) ===
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([worlds_path, world]), '.sdf -r']
        }.items()
    )
    ld.add_action(gazebo)

    # === Spawn Husky ===
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-z', '0.3']
    )
    ld.add_action(robot_spawner)

    # === ROS–Ignition Bridge ===
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml']),
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(gazebo_bridge)

    # === Optional RViz2 ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        condition=IfCondition(rviz)
    )
    ld.add_action(rviz_node)

    # === Optional Nav2 ===
    nav2_include = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path, 'launch', '41068_navigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(nav2)
    )
    ld.add_action(nav2_include)

    # === Bamboo Randomizer (spawns bamboo + publishes /bamboo/targets) ===
    bamboo_randomizer = Node(
        package='ve_behaviours',
        executable='bamboo_randomizer',
        name='bamboo_randomizer',
        output='screen',
        parameters=[{
            'world_name': world,
            'num': 6,
            'min_spacing': 1.8,
            'wall_margin': 1.0,
            'husky_exclusion_radius': 2.5,
            'delete_existing_bamboo': True,
            'seed': bamboo_seed,
        }]
    )
    ld.add_action(TimerAction(period=5.0, actions=[bamboo_randomizer]))

    # === UI (Control Panel) ===
    ui_control = Node(
        package='ve_behaviours',
        executable='ui',
        name='verdant_eye_ui',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(ui_flag)
    )

    # === Auto Wander Depth (Nav2-integrated RGB scanner) ===
    auto_scan = Node(
        package='ve_behaviours',
        executable='auto_wander_depth',
        name='auto_wander_depth',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'goal_frame': 'map',
            'odom_topic': '/odom',
            'camera_topic': '/camera/color/image_raw',
            'targets_topic': '/bamboo/targets',
            'run_enabled_topic': '/ui/run_enabled',
            'restart_srv': '/ui/restart',
            'nav_goal_action': 'navigate_to_pose',
            'standoff_m': 0.6,
            'scan_time_sec': 5.0
        }]
    )

    # Delay UIs + scanner until randomizer finishes spawning
    ld.add_action(TimerAction(period=8.0, actions=[
        ui_control,
        auto_scan
    ]))

    return ld
