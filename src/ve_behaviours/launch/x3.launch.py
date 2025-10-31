#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # --- Args (lowercase boolean strings) ---
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    rviz_arg         = DeclareLaunchArgument('rviz',        default_value='true')
    nav2_arg         = DeclareLaunchArgument('nav2',        default_value='true')
    world_arg        = DeclareLaunchArgument('world',       default_value='large_demo')
    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz_arg)
    ld.add_action(nav2_arg)
    ld.add_action(world_arg)

    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Package shares ---
    this_pkg    = FindPackageShare('ve_behaviours')
    bringup_pkg = FindPackageShare('41068_ignition_bringup')

    # --- Paths (URDF from ve_behaviours; everything else from bringup) ---
    urdf_file = PathJoinSubstitution([this_pkg, 'urdf', 'x3.urdf.xacro'])
    config_dir  = PathJoinSubstitution([bringup_pkg, 'config'])
    worlds_dir  = PathJoinSubstitution([bringup_pkg, 'worlds'])
    world_base  = PathJoinSubstitution([worlds_dir, LaunchConfiguration('world')])  # e.g. .../large_demo
    rviz_file   = PathJoinSubstitution([config_dir, '41068.rviz'])
    rl_yaml     = PathJoinSubstitution([config_dir, 'robot_localization.yaml'])
    bridge_yaml = PathJoinSubstitution([config_dir, 'gazebo_bridge.yaml'])
    nav2_launch = PathJoinSubstitution([bringup_pkg, 'launch', '41068_navigation.launch.py'])

    # --- robot_state_publisher (xacro --inorder) ---
    robot_description = ParameterValue(
    Command(['xacro ', urdf_file]),   # no --inorder; keep the trailing space after 'xacro '
    value_type=str
    )
    ld.add_action(Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    ))

    # --- robot_localization (EKF) from 41068 config ---
    ld.add_action(Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[rl_yaml, {'use_sim_time': use_sim_time}]
    ))

    # --- Ignition Gazebo world from 41068 worlds ---
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py'
        ])),
        # If your launcher supports gz_args, you can swap ign_args -> gz_args.
        launch_arguments={
            'ign_args': [world_base, TextSubstitution(text='.sdf -r')]
        }.items()
    ))

    # --- Spawn the drone from /robot_description ---
    ld.add_action(Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-z', '5.0']
    ))

    # --- ROS <-> Ignition bridge (use -c YAML from 41068 config) ---
    ld.add_action(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-c', bridge_yaml]
    ))

    # --- RViz from 41068 config (optional) ---
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    ))

    # --- Nav2 include FROM 41068_ignition_bringup (optional) ---
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    ))

    return ld
