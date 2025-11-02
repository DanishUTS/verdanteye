# ======================================================================
# file: src/ve_behaviours/launch/bamboo_randomizer.launch.py
# ======================================================================
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # ---------------- Paths (match x3.launch.py layout) ----------------
    pkg_path = FindPackageShare('ve_behaviours')
    bringup_pkg = FindPackageShare('41068_ignition_bringup')

    config_ignition = PathJoinSubstitution([bringup_pkg, 'config'])
    config_x3 = PathJoinSubstitution([pkg_path, 'config'])

    # ---------------- Common CLI args (same as x3.launch.py) ----------------

    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo']
    ))


    # ---------------- Bamboo randomizer (starts after Gazebo is up) ----------------
    # Expose same params you used earlier
    ld.add_action(DeclareLaunchArgument("num", default_value="6"))
    ld.add_action(DeclareLaunchArgument("seed", default_value="0"))
    ld.add_action(DeclareLaunchArgument("min_spacing", default_value="1.8"))
    ld.add_action(DeclareLaunchArgument("wall_margin", default_value="1.0"))
    ld.add_action(DeclareLaunchArgument("husky_exclusion_radius", default_value="2.5"))
    ld.add_action(DeclareLaunchArgument("delete_existing_bamboo", default_value="true"))

    bamboo_spawner = Node(
        package="ve_behaviours",
        executable="bamboo_randomizer",
        name="bamboo_randomizer",
        output="screen",
        parameters=[{
            # world_name uses the selected world
            "world_name": LaunchConfiguration("world"),
            "num": LaunchConfiguration("num"),
            "seed": LaunchConfiguration("seed"),
            "min_spacing": LaunchConfiguration("min_spacing"),
            "wall_margin": LaunchConfiguration("wall_margin"),
            "husky_exclusion_radius": LaunchConfiguration("husky_exclusion_radius"),
            "delete_existing_bamboo": LaunchConfiguration("delete_existing_bamboo"),
        }],
    )

    # Delay to ensure /world/<world>/create service is available
    ld.add_action(TimerAction(period=5.0, actions=[bamboo_spawner]))

    return ld
