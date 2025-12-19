from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
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
        default_value='walls_blocks',  
        description='Gazebo world (walls, walls_blocks, cafe, cafe_table, empty, etc.)'
    )

    declare_path_type = DeclareLaunchArgument(
        'path_type',
        default_value='line',
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
        default_value='map',
        description='Reference frame for trajectory and planning'
    )


    # Controller parameters

    declare_control_rate = DeclareLaunchArgument(
        'control_rate',
        default_value='10.0',
        description='Control loop frequency (Hz)'
    )

    # DUMMY CONTROLLER
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

    # MPC CONTROLLER

    declare_mpc_horizon = DeclareLaunchArgument(
        'mpc_horizon',
        default_value='15',
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


    # MAP related arguments

    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml',
        default_value='walls_blocks_map.yaml',  
        description='Map YAML file (without path; lives in src/maps/)'
    )

    declare_use_map_server = DeclareLaunchArgument(
        'use_map_server',
        default_value='true',
        description='Launch map_server and obstacle publisher'
    )

    declare_occ_threshold = DeclareLaunchArgument(
        'occ_threshold',
        default_value='50',
        description='Occupancy threshold for obstacles (0-100)'
    )

    declare_stride = DeclareLaunchArgument(
        'stride',
        default_value='2',
        description='Downsample map by this factor (1=no downsampling, 2=half resolution)'
    )


    declare_dt = DeclareLaunchArgument('dt', default_value='0.1', description='MPC integration step (s)')
    declare_v_min = DeclareLaunchArgument('v_min', default_value='-0.5', description='Minimum forward velocity (m/s)')
    declare_Q_x = DeclareLaunchArgument('Q_x', default_value='6.0', description='Tracking weight for x')
    declare_Q_y = DeclareLaunchArgument('Q_y', default_value='6.0', description='Tracking weight for y')
    declare_Q_theta = DeclareLaunchArgument('Q_theta', default_value='1.0', description='Tracking weight for theta')
    declare_R_v = DeclareLaunchArgument('R_v', default_value='0.2', description='Effort weight for v')
    declare_R_omega = DeclareLaunchArgument('R_omega', default_value='0.2', description='Effort weight for omega')
    declare_v_ref = DeclareLaunchArgument('v_ref', default_value='0.6', description='Desired cruising speed for MPC (m/s)')
    declare_W_obstacle = DeclareLaunchArgument('W_obstacle', default_value='4.0', description='Obstacle penalty weight')
    declare_d_safe = DeclareLaunchArgument('d_safe', default_value='0.1', description='Extra safety margin around obstacles (m)')
    declare_robot_radius = DeclareLaunchArgument('robot_radius', default_value='0.2', description='Robot radius (m)')
    declare_optimizer_maxiter = DeclareLaunchArgument('optimizer_maxiter', default_value='60', description='SLSQP max iterations')


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
    map_yaml = LaunchConfiguration('map_yaml')
    use_map_server = LaunchConfiguration('use_map_server')
    occ_threshold = LaunchConfiguration('occ_threshold')
    stride = LaunchConfiguration('stride')

    dt = LaunchConfiguration('dt')
    v_min = LaunchConfiguration('v_min')
    Q_x = LaunchConfiguration('Q_x')
    Q_y = LaunchConfiguration('Q_y')
    Q_theta = LaunchConfiguration('Q_theta')
    R_v = LaunchConfiguration('R_v')
    R_omega = LaunchConfiguration('R_omega')
    v_ref = LaunchConfiguration('v_ref')
    W_obstacle = LaunchConfiguration('W_obstacle')
    d_safe = LaunchConfiguration('d_safe')
    robot_radius = LaunchConfiguration('robot_radius')
    optimizer_maxiter = LaunchConfiguration('optimizer_maxiter')


    # Include cafe.launch.py (reusable Gazebo + TIAGo launcher)
    cafe_launch_path = PathJoinSubstitution([
        FindPackageShare('pdm_test'), 'launch', 'cafe.launch.py'
    ])

    cafe_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([cafe_launch_path]),
        launch_arguments={
            'world_name': world_name,
            'run_straight_driver': 'false',
            'is_public_sim': 'True',
        }.items()
    )

    # Trajectory generator node
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
        'mpc_horizon': mpc_horizon,
        'dt': dt,
        'max_v': max_v,
        'v_min': v_min,
        'max_omega': max_omega,
        'Q_x': Q_x,
        'Q_y': Q_y,
        'Q_theta': Q_theta,
        'R_v': R_v,
        'R_omega': R_omega,
        'v_ref': v_ref,
        'W_obstacle': W_obstacle,
        'd_safe': d_safe,
        'robot_radius': robot_radius,
        'optimizer_maxiter': optimizer_maxiter,
        # dummy controller params (still used if controller_type==dummy)
        'k_heading': k_heading,
        'v_const': v_const,
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

    # TODO: This will only need to be in one launch file eventuallys
    # Map server node (from yaml file) (conditional)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': PathJoinSubstitution([
                FindPackageShare('pdm_test'), 'maps', map_yaml
            ])
        }],
        condition=IfCondition(use_map_server)
    )

    # Obstacle publisher node (conditional)
    obstacle_publisher_node = Node(
        package='pdm_test',
        executable='obstacle_publisher',
        name='obstacle_publisher',
        output='screen',
        parameters=[{
            'occ_threshold': occ_threshold,
            'stride': stride,
            'frame_id': frame_id,
            'min_obstacle_radius': 0.15,
        }],
        condition=IfCondition(use_map_server)
    )

    map_server_activate = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 2 && ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate'],
        output='screen',
        condition=IfCondition(use_map_server)
    )

    static_map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(use_map_server)
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
    ld.add_action(declare_map_yaml)
    ld.add_action(declare_use_map_server)
    ld.add_action(declare_occ_threshold)
    ld.add_action(declare_stride)
    ld.add_action(declare_dt)
    ld.add_action(declare_v_min)
    ld.add_action(declare_Q_x)
    ld.add_action(declare_Q_y)
    ld.add_action(declare_Q_theta)
    ld.add_action(declare_R_v)
    ld.add_action(declare_R_omega)
    ld.add_action(declare_v_ref)
    ld.add_action(declare_W_obstacle)
    ld.add_action(declare_d_safe)
    ld.add_action(declare_robot_radius)
    ld.add_action(declare_optimizer_maxiter)

    # Add nodes and launches
    ld.add_action(log_info)
    ld.add_action(cafe_sim)
    ld.add_action(trajectory_node)
    ld.add_action(mpc_node)
    ld.add_action(rviz_node)
    ld.add_action(map_server_node)
    ld.add_action(map_server_activate)
    ld.add_action(obstacle_publisher_node)
    ld.add_action(static_map_odom_tf)
    
    return ld
