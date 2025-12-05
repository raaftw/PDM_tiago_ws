from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Choose which world to load (name without .world extension)
    world_name = LaunchConfiguration('world_name')

    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='empty',
        description=(
            'World name (without .world extension) to load from pdm_test/worlds. '
            'Valid options: cafe, cafe_table, cafe_dynamic, empty'
        )
    )

    # Argument to optionally run the straight_driver node
    declare_run_straight_driver = DeclareLaunchArgument(
        'run_straight_driver',
        default_value='false',
        description='If true, launch the straight_driver node; otherwise just spawn world + TIAGo'
    )

    # Point Gazebo to our package worlds using substitutions so resolution
    # happens at launch time (avoids evaluating substitutions early).
    worlds_dir = PathJoinSubstitution([FindPackageShare('pdm_test'), 'worlds'])

    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=worlds_dir,
    )

    # Path to PAL's TIAGo Gazebo launch file (from tiago_public_ws).
    # Use substitutions so the file path is resolved at launch time.
    tiago_gazebo_launch = PathJoinSubstitution([
        FindPackageShare('tiago_gazebo'), 'launch', 'tiago_gazebo.launch.py'
    ])

    # Include TIAGo simulation, telling it to use our world name.
    tiago_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([tiago_gazebo_launch]),
        launch_arguments={
            'is_public_sim': 'True',
            'world_name': world_name,
        }.items()
    )

    # Launch a simple straight-driving node so the robot moves forward (conditional).
    straight_driver_node = Node(
        package='pdm_test',
        executable='straight_driver',
        name='straight_driver',
        output='screen',
        parameters=[{'linear_speed': 0.2, 'angular_speed': 0.5, 'duration': 60.0}],
        condition=IfCondition(LaunchConfiguration('run_straight_driver')),
    )

    ld = LaunchDescription()
    ld.add_action(declare_world_name)
    ld.add_action(declare_run_straight_driver)
    ld.add_action(set_gazebo_resource_path)
    ld.add_action(straight_driver_node)
    # Add a small info log so we can verify the launch file was processed.
    ld.add_action(LogInfo(msg=['Launching TIAGo sim with world: ', world_name]))
    ld.add_action(tiago_sim)
    return ld
