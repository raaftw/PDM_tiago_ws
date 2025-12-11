from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    """
    Local planner demo launch file.
    
    Launches: Gazebo world + TIAGo + trajectory generator + controller + (optionally) RViz
    
    Arguments:
      - world_name: Gazebo world (default: pal_office)
      - path_type: circle, line, or obstacle_test (default: circle)
      - controller_type: dummy or mpc (default: mpc)
      - use_rviz: true/false (default: true)
    """

    # Declare launch arguments
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='pal_office',
        description='Gazebo world to load (pal_office, cafe, cafe_table, cafe_dynamic, empty)'
    )

    declare_path_type = DeclareLaunchArgument(
        'path_type',
        default_value='circle',
        description='Trajectory type: circle, line, or obstacle_test'
    )

    declare_controller_type = DeclareLaunchArgument(
        'controller_type',
        default_value='mpc',
        description='Controller type: dummy or mpc'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

     # start_x argument
    declare_start_x = DeclareLaunchArgument(
        'start_x',
        default_value='0.0',
        description='Start x coordinate'
    )

    # start_y argument
    declare_start_y = DeclareLaunchArgument(
        'start_y',
        default_value='0.0',
        description='Start y coordinate'
    )

    # goal_x argument
    declare_goal_x = DeclareLaunchArgument(
        'goal_x',
        default_value='5.0',
        description='Goal x coordinate'
    )

    # goal_y argument
    declare_goal_y = DeclareLaunchArgument(
        'goal_y',
        default_value='0.0',
        description='Goal y coordinate'
    )

    # Trajectory parameters
    declare_circle_center_x = DeclareLaunchArgument(
        'circle_center_x',
        default_value='0.0',
        description='Circle center x coordinate'
    )

    declare_circle_center_y = DeclareLaunchArgument(
        'circle_center_y',
        default_value='0.0',
        description='Circle center y coordinate'
    )

    declare_circle_radius = DeclareLaunchArgument(
        'circle_radius',
        default_value='1.5',
        description='Circle radius (meters)'
    )

    declare_start_angle = DeclareLaunchArgument(
        'start_angle',
        default_value='0.0',
        description='Starting angle for circle path (radians)'
    )

    declare_direction = DeclareLaunchArgument(
        'direction',
        default_value='ccw',
        description='Circle direction: ccw or cw'
    )

    declare_frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='odom',
        description='Reference frame for trajectory and planning'
    )

    # Controller parameters
    declare_k_heading = DeclareLaunchArgument(
        'k_heading',
        default_value='5.0',
        description='Heading controller gain (dummy controller)'
    )

    declare_v_const = DeclareLaunchArgument(
        'v_const',
        default_value='0.3',
        description='Constant forward velocity (m/s)'
    )

    declare_control_rate = DeclareLaunchArgument(
        'control_rate',
        default_value='10.0',
        description='Control loop frequency (Hz)'
    )

    declare_mpc_horizon = DeclareLaunchArgument(
        'mpc_horizon',
        default_value='10',
        description='MPC prediction horizon (steps)'
    )

    declare_max_v = DeclareLaunchArgument(
        'max_v',
        default_value='5.0',
        description='Maximum forward velocity (m/s)'
    )

    declare_max_omega = DeclareLaunchArgument(
        'max_omega',
        default_value='5.0',
        description='Maximum angular velocity (rad/s)'
    )


    # Launch configurations
    world_name = LaunchConfiguration('world_name')
    path_type = LaunchConfiguration('path_type')
    controller_type = LaunchConfiguration('controller_type')
    use_rviz = LaunchConfiguration('use_rviz')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    circle_center_x = LaunchConfiguration('circle_center_x')
    circle_center_y = LaunchConfiguration('circle_center_y')
    circle_radius = LaunchConfiguration('circle_radius')
    start_angle = LaunchConfiguration('start_angle')
    direction = LaunchConfiguration('direction')
    frame_id = LaunchConfiguration('frame_id')
    k_heading = LaunchConfiguration('k_heading')
    v_const = LaunchConfiguration('v_const')
    control_rate = LaunchConfiguration('control_rate')
    mpc_horizon = LaunchConfiguration('mpc_horizon')
    max_v = LaunchConfiguration('max_v')
    max_omega = LaunchConfiguration('max_omega')

    # Include cafe.launch.py (reusable Gazebo + TIAGo launcher)
    cafe_launch_path = PathJoinSubstitution([
        FindPackageShare('pdm_test'), 'launch', 'cafe.launch.py'
    ])

    cafe_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([cafe_launch_path]),
        launch_arguments={
            'world_name': world_name,
            'run_straight_driver': 'false',
        }.items()
    )

    # Trajectory generator node
    # Note: Parameters vary by path_type - you can extend this with more sophisticated
    # parameter passing if needed (e.g., using OpaqueFunction)
    trajectory_node = Node(
        package='pdm_test',
        executable='trajectory_generator',
        name='trajectory_generator',
        output='screen',
        parameters=[{
            'path_type': path_type,
            'frame_id': frame_id,
            'publish_rate': 1.0,
            'num_points': 100,
            'start_x': start_x,
            'start_y': start_y,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'circle_center_x': circle_center_x,
            'circle_center_y': circle_center_y,
            'circle_radius': circle_radius,
            'start_angle': start_angle,
            'direction': direction,
        }],
    )

    # MPC controller node
    mpc_node = Node(
        package='pdm_test',
        executable='mpc_controller',
        name='mpc_controller',
        output='screen',
        parameters=[{
            'controller_type': controller_type,
            'control_rate': control_rate,
            'k_heading': k_heading,
            'v_const': v_const,
            'mpc_horizon': mpc_horizon,
            'dt': 0.1,
            'max_v': max_v,
            'max_omega': max_omega,
            'v_min': -0.5,
            'Q_x': 10.0,
            'Q_y': 10.0,
            'Q_theta': 5.0,
            'R_v': 0.1,
            'R_omega': 0.1,
        }],
    )

    # RViz node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-f', frame_id],
        condition=IfCondition(use_rviz)
    )

    # Info log
    log_info = LogInfo(
        msg=['Launching local planner demo: world=', world_name, 
             ', path=', path_type, ', controller=', controller_type]
    )

    # Build launch description
    ld = LaunchDescription()
    
    # Add argument declarations
    ld.add_action(declare_world_name)
    ld.add_action(declare_path_type)
    ld.add_action(declare_controller_type)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_start_x)
    ld.add_action(declare_start_y)
    ld.add_action(declare_goal_x)
    ld.add_action(declare_goal_y)
    ld.add_action(declare_circle_center_x)
    ld.add_action(declare_circle_center_y)
    ld.add_action(declare_circle_radius)
    ld.add_action(declare_start_angle)
    ld.add_action(declare_direction)
    ld.add_action(declare_frame_id)
    ld.add_action(declare_k_heading)
    ld.add_action(declare_v_const)
    ld.add_action(declare_control_rate)
    ld.add_action(declare_mpc_horizon)
    ld.add_action(declare_max_v)
    ld.add_action(declare_max_omega)

    # Add nodes and launches
    ld.add_action(log_info)
    ld.add_action(cafe_sim)
    ld.add_action(trajectory_node)
    ld.add_action(mpc_node)
    ld.add_action(rviz_node)
    
    return ld
