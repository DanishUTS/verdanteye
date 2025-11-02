#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
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
    launches_path = PathJoinSubstitution([pkg_path, 'launch'])

    # === Env: make sure Gazebo/Ignition can resolve model:// URIs ===
    ld.add_action(SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_path, os.pathsep, os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    ))
    ld.add_action(SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[models_path, os.pathsep, os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]
    ))
    ld.add_action(SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[models_path, os.pathsep, os.environ.get('GAZEBO_MODEL_PATH', '')]
    ))

    # === Launch args ===
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation clock'
    )
    ld.add_action(use_sim_time_launch_arg)
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='False', description='Launch RViz2'
    )
    ld.add_action(rviz_launch_arg)

    nav2_launch_arg = DeclareLaunchArgument(
        'nav2', default_value='True', description='Launch Nav2'
    )
    ld.add_action(nav2_launch_arg)

    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo']
    )
    ld.add_action(world_launch_arg)

    # === Robot description ===
    robot_description_content = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf', 'husky.urdf.xacro'])]),
        value_type=str
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_state_publisher_node)

    # === robot_localization ===
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([config_path, 'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_localization_node)

    # === Gazebo (Ignition) world ===
    # Fix: ensure the '.sdf' extension is included correctly
    world_path_no_ext = PathJoinSubstitution([pkg_path, 'worlds', LaunchConfiguration('world')])
    ign_args = [world_path_no_ext, TextSubstitution(text='.sdf -r')]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py'])
        ),
        launch_arguments={'ign_args': ign_args}.items()
    )
    ld.add_action(gazebo)

    # === Spawn robot ===
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-z', '0.4']
    )
    ld.add_action(robot_spawner)

    # === Bridge ===
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml']),
                    'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)

    # === RViz (optional) ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # === Nav2 (optional) ===
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([launches_path, '41068_navigation.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    # === Bamboo randomizer (starts a few seconds after world is up) ===
    bamboo_randomizer = Node(
        package='ve_behaviours',
        executable='bamboo_randomizer',
        name='bamboo_randomizer',
        output='screen',
        parameters=[{
            'world_name': LaunchConfiguration('world'),
            'num': 6,
            'min_spacing': 1.8,
            'wall_margin': 1.0,
            'husky_exclusion_radius': 2.5,
            'delete_existing_bamboo': True,
            # Z is handled inside the node; default 0.0
        }]
    )
    # Give Gazebo a moment to advertise /create service on slower machines
    ld.add_action(TimerAction(period=6.0, actions=[bamboo_randomizer]))

    return ld
