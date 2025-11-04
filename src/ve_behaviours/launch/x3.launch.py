from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    # Get paths to directories
    pkg_path = FindPackageShare('ve_behaviours')
    bringup_pkg = FindPackageShare('41068_ignition_bringup')

    config_ignition = PathJoinSubstitution([bringup_pkg,
                                       'config'])
    
    config_x3 = PathJoinSubstitution([pkg_path,
                                       'config'])

    # Additional command line arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)
    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2'
    )
    ld.add_action(nav2_launch_arg)

    # Load robot_description and start robot_state_publisher
    robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path,
                                       'urdf',
                                       'x3.urdf.xacro'])]),
        value_type=str)
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                          'use_sim_time': use_sim_time
                                      }])
    ld.add_action(robot_state_publisher_node)

    # Publish odom -> base_link transform **using robot_localization**
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([config_x3,
                                          'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_localization_node)

    # Start Gazebo to simulate the robot in the chosen world
    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo']
    )
    ld.add_action(world_launch_arg)
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                             'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([bringup_pkg,
                                               'worlds',
                                               [LaunchConfiguration('world'), '.sdf']]),
                         ' -r']}.items()
    )
    ld.add_action(gazebo)

    # Spawn robot in Gazebo
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-x', '-11.0', '-y', '11.0', '-z', '2.0'] # z is height above ground
    )
    ld.add_action(robot_spawner)

    # Bridge topics between gazebo and ROS2
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_x3,
                                                          'gazebo_bridge.yaml']),
                    'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)

    # rviz2 visualises data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_x3,
                                               '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # Nav2 enables mapping and waypoint following
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([bringup_pkg,
                              'launch',
                              '41068_navigation.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    depth_to_meters = Node(
    package='depth_image_proc',
    executable='convert_metric_node',
    name='depth_convert_metric',
    remappings=[
        ('image_raw', '/X3/rgbd_camera/depth_image'),          # in
        ('image',     '/X3/rgbd_camera/depth_image_meters')    # out (32FC1 meters)
    ],
    parameters=[{'use_sim_time': use_sim_time}]
    )
    ld.add_action(depth_to_meters)

    depth_to_scan = Node(
    package='depthimage_to_laserscan',
    executable='depthimage_to_laserscan_node',
    name='depth_to_scan',
    parameters=[{
        'output_frame_id': 'base_scan',     # or camera frame if you prefer
        'range_min': 0.3,
        'range_max': 8.0,
        'scan_time': 0.1
    }],
    remappings=[
        ('depth',            '/X3/rgbd_camera/depth_image'),
        ('depth_camera_info','/X3/rgbd_camera/camera_info'),
        ('scan',             '/scan')
    ]
    )
    ld.add_action(depth_to_scan)

    # Enable / disable RGB-D SLAM from CLI: slam_rgbd:=true|false
    slam_rgbd_arg = DeclareLaunchArgument(
        'slam_rgbd', default_value='True',
        description='Run RTAB-Map RGB-D SLAM (publishes /map and map->odom)'
    )
    ld.add_action(slam_rgbd_arg)

    # 1) RGB+Depth synchronizer
    rgbd_sync = Node(
        package='rtabmap_sync',                 # <— was rtabmap_ros
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('rgb/image',       '/X3/rgbd_camera/image'),
            ('depth/image',     '/X3/rgbd_camera/depth_image'),
            ('rgb/camera_info', '/X3/rgbd_camera/camera_info'),
            ('rgbd_image',      '/rgbd_image')
        ],
        condition=IfCondition(LaunchConfiguration('slam_rgbd'))
    )
    ld.add_action(rgbd_sync)

    # 2) RTAB-Map SLAM core
    rtabmap = Node(
        package='rtabmap_slam',                 # <— was rtabmap_ros
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'frame_id': 'base_link'},
            {'odom_frame_id': 'odom'},
            {'map_frame': 'map'},
            {'subscribe_depth': True},
            {'approx_sync': True},
            {'queue_size': 30},
            {'Grid/FromDepth': 'true'},
            {'Grid/RangeMax': '12.0'},
            {'Grid/MinClusterSize': '10'},
            {'RGBD/LoopClosureReextractFeatures': 'false'},
            {'Optimizer/Strategy': '1'}
        ],
        remappings=[
            ('rgbd_image', '/rgbd_image')
        ],
        condition=IfCondition(LaunchConfiguration('slam_rgbd'))
    )
    ld.add_action(rtabmap)

    return ld
