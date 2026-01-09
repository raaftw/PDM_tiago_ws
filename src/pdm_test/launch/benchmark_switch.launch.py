from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


"""Toggle between Nav2 stack and the existing MPC stack in one launch.

Usage examples:
  ros2 launch pdm_test benchmark_switch.launch.py use_nav2:=false world_name:=walls_blocks
  ros2 launch pdm_test benchmark_switch.launch.py use_nav2:=true world_name:=walls_blocks
"""


def generate_launch_description():
    use_nav2 = LaunchConfiguration('use_nav2')
    nav2_mode = LaunchConfiguration('nav2_mode')
    world_name = LaunchConfiguration('world_name')
    map_arg = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('nav2_params')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    record_bag = LaunchConfiguration('record_bag')
    bag_prefix = LaunchConfiguration('bag_prefix')

    declare_use_nav2 = DeclareLaunchArgument(
        'use_nav2',
        default_value='false',
        description='Switch between Nav2 (true) and MPC stack (false)'
    )

    declare_nav2_mode = DeclareLaunchArgument(
        'nav2_mode',
        default_value='tiago_public',
        choices=['tiago_public', 'direct'],
        description='Nav2 launch mode: tiago_public uses tiago_gazebo with built-in nav2, direct uses nav2_bringup'
    )

    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='walls_blocks',
        description='World name without extension (e.g., cafe, walls_blocks)'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('pdm_test'),
            'maps',
            PythonExpression(["'", world_name, "'", " + '_map.yaml'"])
        ]),
        description='Full path to map YAML file'
    )

    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'params',
            'nav2_params.yaml'
        ]),
        description='Nav2 params YAML file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz when running Nav2 branch'
    )

    declare_record_bag = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        description='Enable ros2 bag recording of benchmark topics'
    )

    declare_bag_prefix = DeclareLaunchArgument(
        'bag_prefix',
        default_value='benchmark_bag',
        description='Output prefix for rosbag2'
    )

    declare_goal_location = DeclareLaunchArgument(
        'goal_location',
        default_value='center',
        choices=['center', 'corner_1', 'corner_2', 'corner_3', 'corner_4'],
        description='Predefined goal location: center, or corner_1/2/3/4 in front of tables'
    )

    declare_goal_x = DeclareLaunchArgument(
        'goal_x',
        default_value='',
        description='Goal X coordinate (overrides goal_location if set)'
    )

    declare_goal_y = DeclareLaunchArgument(
        'goal_y',
        default_value='',
        description='Goal Y coordinate (overrides goal_location if set)'
    )

    declare_goal_theta = DeclareLaunchArgument(
        'goal_theta',
        default_value='',
        description='Goal orientation (overrides goal_location if set)'
    )

    declare_goal_delay = DeclareLaunchArgument(
        'goal_delay',
        default_value='10.0',
        description='Seconds to wait before publishing goal (for startup)'
    )

    goal_location = LaunchConfiguration('goal_location')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_theta = LaunchConfiguration('goal_theta')
    goal_delay = LaunchConfiguration('goal_delay')

    # ----- MPC branch (existing combined launch) -----
    mpc_combined_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pdm_test'),
                'launch',
                'mpc_combined.launch.py'
            ])
        ]),
        launch_arguments={
            'world_name': world_name,
            'map': map_arg,
        }.items(),
        condition=UnlessCondition(use_nav2)
    )

    # ----- Nav2 branch: tiago_public mode (external TIAGo sim with nav2) -----
    from launch.conditions import LaunchConfigurationEquals
    
    nav2_tiago_public = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'tiago_gazebo', 'tiago_gazebo.launch.py',
            ['is_public_sim:=True'],
            ['world_name:=', world_name],
            ['navigation:=True'],
            ['slam:=True']
        ],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", use_nav2, "' == 'true' and '", nav2_mode, "' == 'tiago_public'"
        ]))
    )

    # ----- Nav2 branch: direct mode (our cafe.launch + nav2_bringup) -----
    cafe_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pdm_test'),
                'launch',
                'cafe.launch.py'
            ])
        ]),
        launch_arguments={
            'world_name': world_name,
            'run_straight_driver': 'false',
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", use_nav2, "' == 'true' and '", nav2_mode, "' == 'direct'"
        ]))
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': map_arg,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
            'use_rviz': use_rviz,
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", use_nav2, "' == 'true' and '", nav2_mode, "' == 'direct'"
        ]))
    )

    ground_truth_republisher = Node(
        package='pdm_test',
        executable='ground_truth_republisher',
        name='ground_truth_republisher',
        output='screen',
        condition=IfCondition(use_nav2)
    )

    # Record key topics for offline comparison
    record_topics = [
        '/ground_truth_odom',
        '/reference_path',
        '/cmd_vel',
        '/scan_raw',
        '/goal_pose',
        '/metrics/min_obstacle_distance',
        '/metrics/path_lateral_error',
        '/tf',
        '/tf_static'
    ]

    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_prefix] + record_topics,
        output='screen',
        condition=IfCondition(record_bag)
    )

    # Metrics node for Nav2 (compute min obstacle distance from scan)
    metrics_min_dist_node = Node(
        package='pdm_test',
        executable='metrics_min_distance',
        name='metrics_min_distance',
        output='screen',
        parameters=[{
            'scan_topics': ['/scan_raw', '/scan', '/front_laser/scan']
        }]
    )

    # Automatic goal publisher for consistent benchmarking
    # For MPC: publishes to /goal_pose
    # For Nav2: sends NavigateToPose action
    goal_publisher_mpc = Node(
        package='pdm_test',
        executable='goal_publisher',
        name='goal_publisher',
        output='screen',
        parameters=[{
            'mode': 'mpc',
            'goal_location': goal_location,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_theta': goal_theta,
            'startup_delay': goal_delay,
            'frame_id': 'map',
        }],
        condition=UnlessCondition(use_nav2)
    )

    goal_publisher_nav2 = Node(
        package='pdm_test',
        executable='goal_publisher',
        name='goal_publisher',
        output='screen',
        parameters=[{
            'mode': 'nav2',
            'goal_location': goal_location,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_theta': goal_theta,
            'startup_delay': goal_delay,
            'frame_id': 'map',
        }],
        condition=IfCondition(use_nav2)
    )

    ld = LaunchDescription()

    # Arguments
    ld.add_action(declare_use_nav2)
    ld.add_action(declare_nav2_mode)
    ld.add_action(declare_world_name)
    ld.add_action(declare_map)
    ld.add_action(declare_nav2_params)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_record_bag)
    ld.add_action(declare_bag_prefix)
    ld.add_action(declare_goal_location)
    ld.add_action(declare_goal_x)
    ld.add_action(declare_goal_y)
    ld.add_action(declare_goal_theta)
    ld.add_action(declare_goal_delay)

    # Branches
    ld.add_action(mpc_combined_launch)
    ld.add_action(nav2_tiago_public)
    ld.add_action(cafe_launch)
    ld.add_action(nav2_bringup)
    ld.add_action(ground_truth_republisher)
    ld.add_action(metrics_min_dist_node)
    ld.add_action(goal_publisher_mpc)
    ld.add_action(goal_publisher_nav2)
    ld.add_action(bag_record)

    return ld
