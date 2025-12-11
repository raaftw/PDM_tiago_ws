from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    map_name = LaunchConfiguration('map_name')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    occ_threshold = LaunchConfiguration('occ_threshold')
    clearance_m = LaunchConfiguration('clearance_m')
    step_size = LaunchConfiguration('step_size')
    goal_tol = LaunchConfiguration('goal_tol')
    max_iters = LaunchConfiguration('max_iters')
    rewire_radius = LaunchConfiguration('rewire_radius')

    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value='walls_map',
        description='Map name (without extension) to load from src/maps'
    )

    declare_goal_x = DeclareLaunchArgument(
        'goal_x',
        default_value='2.0',
        description='Goal position X coordinate'
    )

    declare_goal_y = DeclareLaunchArgument(
        'goal_y',
        default_value='0.0',
        description='Goal position Y coordinate'
    )

    declare_occ_threshold = DeclareLaunchArgument(
        'occ_threshold',
        default_value='50',
        description='Occupancy threshold (0-100) to treat as obstacle'
    )

    declare_clearance = DeclareLaunchArgument(
        'clearance_m',
        default_value='0.3',
        description='Clearance (meters) to inflate obstacles for planning'
    )

    declare_step_size = DeclareLaunchArgument(
        'step_size',
        default_value='0.5',
        description='RRT* step size in meters'
    )

    declare_goal_tol = DeclareLaunchArgument(
        'goal_tol',
        default_value='0.3',
        description='RRT* goal tolerance in meters'
    )

    declare_max_iters = DeclareLaunchArgument(
        'max_iters',
        default_value='2000',
        description='RRT* maximum iterations'
    )

    declare_rewire_radius = DeclareLaunchArgument(
        'rewire_radius',
        default_value='1.0',
        description='RRT* rewire radius in meters'
    )

    # Path to map YAML file (installed to share/pdm_test/maps)
    map_yaml = PathJoinSubstitution([
        FindPackageShare('pdm_test'), 'maps', [map_name, '.yaml']
    ])

    # Simple map publisher node
    map_publisher = Node(
        package='pdm_test',
        executable='simple_map_publisher',
        name='map_publisher',
        parameters=[{
            'map_yaml': map_yaml,
        }],
        output='screen'
    )

    # Straight line planner node (robot assumed at 0,0)
    planner_node = Node(
        package='pdm_test',
        executable='rrt_star_planner',
        name='rrt_star_planner',
        parameters=[{
            'map_topic': '/map',
            'path_topic': '/global_path',
            'goal_x': goal_x,
            'goal_y': goal_y,
            'occ_threshold': occ_threshold,
            'clearance_m': clearance_m,
            'step_size': step_size,
            'goal_tolerance': goal_tol,
            'max_iters': max_iters,
            'rewire_radius': rewire_radius,
        }],
        output='screen'
    )

    return LaunchDescription([
        declare_map_name,
        declare_goal_x,
        declare_goal_y,
        declare_occ_threshold,
        declare_clearance,
        declare_step_size,
        declare_goal_tol,
        declare_max_iters,
        declare_rewire_radius,
        map_publisher,
        planner_node,
    ])
