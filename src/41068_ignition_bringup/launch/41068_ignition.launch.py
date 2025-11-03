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
    for env_var in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH', 'GAZEBO_MODEL_PATH'):
        ld.add_action(SetEnvironmentVariable(
            name=env_var,
            value=[models_path, os.pathsep, os.environ.get(env_var, '')]
        ))

    # === Launch args ===
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    nav2 = LaunchConfiguration('nav2')
    world = LaunchConfiguration('world')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation clock'))
    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='False', description='Launch RViz2 (requires ros-humble-rviz2)'))
    # IMPORTANT: default Nav2 OFF so our scanner is the sole /cmd_vel owner
    ld.add_action(DeclareLaunchArgument(
        'nav2', default_value='False', description='Launch Nav2'))
    ld.add_action(DeclareLaunchArgument(
        'world', default_value='simple_trees',
        choices=['simple_trees', 'large_demo'],
        description='Which world to load'))

    # === Robot description ===
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf', 'husky.urdf.xacro'])]),
        value_type=str
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_state_publisher_node)

    # === robot_localization (EKF) ===
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
    world_path_no_ext = PathJoinSubstitution([pkg_path, 'worlds', world])
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

    # === ROS–Ignition bridge ===
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
        condition=IfCondition(rviz)
    )
    ld.add_action(rviz_node)

    # === Nav2 (optional — default OFF) ===
    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([launches_path, '41068_navigation.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(nav2)
    )
    ld.add_action(nav2_include)

    # === Bamboo randomizer (spawns and publishes /bamboo/targets) ===
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
        }]
    )
    # Give Gazebo time to advertise services on slower machines
    ld.add_action(TimerAction(period=6.0, actions=[bamboo_randomizer]))

    # === Control UI ===
    ui_control = Node(
        package='ve_behaviours',
        executable='ui',
        name='verdant_eye_ui',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # === Checklist UI ===
    ui_plants = Node(
        package='ve_behaviours',
        executable='ui_plants',
        name='verdant_eye_gallery',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # === Auto scanner (sole /cmd_vel owner) ===
    auto_scan = Node(
    package='ve_behaviours',
    executable='auto_wander_depth',
    name='auto_wander_depth',
    output='screen',
    emulate_tty=True,
    parameters=[{
        'use_sim_time': use_sim_time,
        'world_name': world,
        'entity_name': 'husky',
        'origin_xy': [0.0, 0.0],
        'invert_angular': True,    # force turn direction flip
        'angular_gain': 1.0,       # smoother rotation
        'linear_gain': 0.6         # less aggressive
    }]
)

    # Start UIs + scanner after the randomizer
    ld.add_action(TimerAction(period=10.0, actions=[
        ui_control,
        ui_plants,
        auto_scan
    ]))

    return ld
