from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """
    Unified launch file combining 3 separate commands:
    
    1. ros2 launch pdm_test cafe.launch.py world_name:=walls_blocks
    2. ros2 launch pdm_test global_planner_demo.launch.xml map:=$(ros2 pkg prefix pdm_test)/share/pdm_test/maps/walls_blocks_map.yaml
    3. ros2 run pdm_test mpc_controller --ros-args -p controller_type:=mpc -p v_const:=0.3 -p k_heading:=5.0
    
    Usage:
        ros2 launch pdm_test mpc_combined_demo.launch.py world_name:=walls_blocks
    """

    # ==================== ARGUMENTS ====================
    
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='cafe',
        description='World name (without .world extension). Map will be [worldname]_map.yaml'
    )

    declare_controller_type = DeclareLaunchArgument(
        'controller_type',
        default_value='mpc',
        description='MPC controller type'
    )

    declare_v_const = DeclareLaunchArgument(
        'v_const',
        default_value='0.3',
        description='Constant forward velocity (m/s)'
    )

    declare_k_heading = DeclareLaunchArgument(
        'k_heading',
        default_value='5.0',
        description='Heading controller gain'
    )

    world_name = LaunchConfiguration('world_name')
    map_arg = LaunchConfiguration('map')
    controller_type = LaunchConfiguration('controller_type')
    v_const = LaunchConfiguration('v_const')
    k_heading = LaunchConfiguration('k_heading')

    # Build map path exactly like the XML launch would resolve:
    # $(find-pkg-share pdm_test)/maps/[world_name]_map.yaml
    # Match XML: expose a `map` arg with default to cafe_map.yaml
    declare_map = DeclareLaunchArgument(
        'map',
        # Default to $(find-pkg-share pdm_test)/maps/[world_name]_map.yaml
        default_value=PathJoinSubstitution([
            FindPackageShare('pdm_test'),
            'maps',
            PythonExpression([
                "'" , world_name, "' + '_map.yaml'"
            ])
        ]),
        description='Full path to map YAML file'
    )

    # ==================== COMMAND 1: Gazebo with TIAGo ====================
    # ros2 launch pdm_test cafe.launch.py world_name:=walls_blocks
    
    cafe_launch_path = PathJoinSubstitution([
        FindPackageShare('pdm_test'), 'launch', 'cafe.launch.py'
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([cafe_launch_path]),
        launch_arguments={
            'world_name': world_name,
            'run_straight_driver': 'false',
        }.items()
    )

    # ==================== COMMAND 2: Global Planner + RViz ====================
    # ros2 launch pdm_test global_planner_demo.launch.xml map:=$(ros2 pkg prefix pdm_test)/share/pdm_test/maps/walls_blocks_map.yaml

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_arg}],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            # Proper list type so lifecycle manager can activate the node
            'node_names': ['map_server']
        }],
    )

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    global_planner_node = Node(
        package='pdm_test',
        executable='global_planner',
        name='rrt_star_planner_node',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('pdm_test'), 'launch', 'global_planner_demo.rviz'
        ])],
    )

    # ==================== COMMAND 3: MPC Controller ====================
    # ros2 run pdm_test mpc_controller --ros-args -p controller_type:=mpc -p v_const:=0.3 -p k_heading:=5.0

    mpc_node = Node(
        package='pdm_test',
        executable='mpc_controller',
        name='mpc_controller',
        output='screen',
        parameters=[{
            'controller_type': controller_type,
            'v_const': v_const,
            'k_heading': k_heading,
        }],
    )

    # ==================== BUILD LAUNCH DESCRIPTION ====================

    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_world_name)
    ld.add_action(declare_map)
    ld.add_action(declare_controller_type)
    ld.add_action(declare_v_const)
    ld.add_action(declare_k_heading)

    # Add command 1: Gazebo
    ld.add_action(gazebo_launch)

    # Add command 2: Global Planner + RViz
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(static_tf_map_odom)
    ld.add_action(global_planner_node)
    ld.add_action(rviz_node)

    # Add command 3: MPC Controller
    ld.add_action(mpc_node)

    return ld
