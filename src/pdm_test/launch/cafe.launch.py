import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Choose which world to load (name without .world extension)
    world_name = LaunchConfiguration('world_name')

    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='cafe',  # options: cafe, cafe_table, cafe_dynamic
        description=(
            'World name (without .world extension) to load from pdm_test/worlds. '
            'Valid options: cafe, cafe_table, cafe_dynamic'
        )
    )

    # Get this package's share/worlds directory
    pdm_share = get_package_share_directory('pdm_test')
    worlds_dir = os.path.join(pdm_share, 'worlds')

    # Make sure Gazebo can find our worlds by name (cafe, cafe_table, etc.)
    # We prepend our worlds_dir to any existing GAZEBO_RESOURCE_PATH.
    old_gazebo_resource = os.getenv('GAZEBO_RESOURCE_PATH', '')
    new_gazebo_resource = worlds_dir if not old_gazebo_resource \
        else worlds_dir + os.pathsep + old_gazebo_resource

    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=new_gazebo_resource,
    )

    # Path to PAL's TIAGo Gazebo launch file (from tiago_public_ws)
    tiago_gazebo_share = get_package_share_directory('tiago_gazebo')
    tiago_gazebo_launch = os.path.join(
        tiago_gazebo_share, 'launch', 'tiago_gazebo.launch.py'
    )

    # Include TIAGo simulation, telling it to use our world name
    tiago_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tiago_gazebo_launch),
        launch_arguments={
            # public simulation is required when using tiago_public_ws
            'is_public_sim': 'True',
            # This is how PAL selects worlds, e.g. world_name:=pal_office
            'world_name': world_name,
            # You can also add navigation/moveit flags later if you want:
            # 'navigation': 'True',
            # 'moveit': 'True',
        }.items()
    )

    return LaunchDescription([
        declare_world_name,
        set_gazebo_resource_path,
        tiago_sim,
    ])
