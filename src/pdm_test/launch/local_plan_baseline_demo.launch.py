from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    # Declare launch arguments
    
    # world_name argument 
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='empty',
        description='World to load in the simulation'
    )

    # path_type argument 
    declare_path_type = DeclareLaunchArgument(
        'path_type',
        default_value='circle',
        description='Path type: line or circle'
    )

    # k_heading argument 
    declare_k_heading = DeclareLaunchArgument(
        'k_heading',
        default_value='5.0',
        description='Heading controller gain'
    )

    # v_const argument 
    declare_v_const = DeclareLaunchArgument(
        'v_const',
        default_value='0.3',
        description='Constant forward velocity (m/s)'
    )

    world_name = LaunchConfiguration('world_name')
    path_type = LaunchConfiguration('path_type')
    k_heading = LaunchConfiguration('k_heading')
    v_const = LaunchConfiguration('v_const')


    cafe_launch_path = PathJoinSubstitution([
        FindPackageShare('pdm_test'), 'launch', 'cafe.launch.py'
    ])

    cafe_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([cafe_launch_path]),
        launch_arguments={
            'world_name': world_name,
            'run_straight_driver': 'false',  # disable straight driver
        }.items()
    )

    trajectory_node = Node(
        package='pdm_test',
        executable='trajectory_generator',
        name='trajectory_generator',
        output='screen',
        parameters=[{
            'path_type': path_type,
            'start_x': 0.0,
            'start_y': 0.0,
            'goal_x': 3.0,
            'goal_y': 0.0,
            'circle_center_x': 0.0,
            'circle_center_y': 0.0,
            'circle_radius': 1.0,
            'frame_id': 'odom',
            'publish_rate': 1.0,
        }],
    )

    mpc_node = Node(
        package='pdm_test',
        executable='mpc_controller',
        name='mpc_controller',
        output='screen',
        parameters=[{
            'control_rate': 10.0,
            'k_heading': k_heading,
            'v_const': v_const,
            'max_v': 5.0,
            'max_omega': 5.0,
        }],
    )

    # Launch RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-f', 'odom']
        # arguments=['-d', PathJoinSubstitution([FindPackageShare('pdm_test'), 'launch', 'mpc_baseline_demo.rviz'])]
    )

    log_info = LogInfo(msg=[
        'Launching baseline MPC path follower: ',
        'world=', world_name, ' path_type=', path_type,
        ' k_heading=', k_heading, ' v=', v_const
    ])

    ld = LaunchDescription()
    # Add argument declarations
    ld.add_action(declare_world_name)
    ld.add_action(declare_path_type)
    ld.add_action(declare_k_heading)
    ld.add_action(declare_v_const)

    # Add nodes/launches
    ld.add_action(cafe_sim)
    ld.add_action(trajectory_node)
    ld.add_action(mpc_node)
    ld.add_action(log_info)
    ld.add_action(rviz_node)

    return ld

