"""
Table Detector Launch File
Launches the table detector node to analyze costmap and detect tables.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('pdm_test')
    
    # Declare arguments
    occupied_threshold_arg = DeclareLaunchArgument(
        'occupied_threshold',
        default_value='65',
        description='Occupancy threshold for detecting obstacles'
    )
    
    min_area_arg = DeclareLaunchArgument(
        'min_table_area',
        default_value='0.15',
        description='Minimum table area in m^2'
    )
    
    max_area_arg = DeclareLaunchArgument(
        'max_table_area',
        default_value='1.0',
        description='Maximum table area in m^2'
    )
    
    save_yaml_arg = DeclareLaunchArgument(
        'save_to_yaml',
        default_value='True',
        description='Save detected tables to YAML file'
    )

    erode_arg = DeclareLaunchArgument(
        'erode_iterations',
        default_value='0',
        description='Number of erosion iterations to trim inflated edges'
    )
    
    yaml_path_arg = DeclareLaunchArgument(
        'yaml_output_path',
        default_value=os.path.join(pkg_share, 'maps', 'tables_detected.yaml'),
        description='Path to save detected tables YAML'
    )
    
    # Table detector node
    table_detector_node = Node(
        package='pdm_test',
        executable='table_detector',
        name='table_detector',
        output='screen',
        parameters=[{
            'occupied_threshold': LaunchConfiguration('occupied_threshold'),
            'min_table_area': LaunchConfiguration('min_table_area'),
            'max_table_area': LaunchConfiguration('max_table_area'),
            'save_to_yaml': LaunchConfiguration('save_to_yaml'),
            'yaml_output_path': LaunchConfiguration('yaml_output_path'),
            'erode_iterations': LaunchConfiguration('erode_iterations'),
        }]
    )
    
    return LaunchDescription([
        occupied_threshold_arg,
        min_area_arg,
        max_area_arg,
        save_yaml_arg,
        erode_arg,
        yaml_path_arg,
        table_detector_node,
    ])
