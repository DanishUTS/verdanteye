from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    with_drone = LaunchConfiguration('with_drone')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('with_drone', default_value='False'),
        DeclareLaunchArgument('rviz', default_value='True'),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'),
                                  'launch','41068_ignition.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'rviz': rviz}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'),
                                  'launch','41068_navigation.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'),
                                  'launch','41068_ignition_drone.launch.py']),
            condition=IfCondition(with_drone),
            launch_arguments={'use_sim_time': use_sim_time, 'rviz': 'False'}.items()),
    ])
