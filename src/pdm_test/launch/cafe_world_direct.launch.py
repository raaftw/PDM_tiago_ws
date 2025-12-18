from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression
from launch.actions import TimerAction

def generate_launch_description():
    world_name = LaunchConfiguration('world_name')

    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='empty',
        description='World name (without .world) from pdm_test/worlds'
    )

    # pdm_test/worlds/<world_name>.world
    worlds_dir = PathJoinSubstitution([FindPackageShare('pdm_test'), 'worlds'])
    world_file = PythonExpression(["'", worlds_dir, "/", world_name, ".world'"])
    # world_file = PathJoinSubstitution([worlds_dir, world_name, TextSubstitution(text='.world')])

    # Make Gazebo aware of our worlds directory (optional but nice)
    '''
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=worlds_dir
    )
    '''

    # 1) Launch Gazebo with the chosen world file
    gazebo_launch = PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )

    # 2) Spawn TIAGo into the already running Gazebo
    robot_spawn_launch = PathJoinSubstitution([FindPackageShare('tiago_gazebo'), 'launch', 'robot_spawn.launch.py'])
    spawn_tiago = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments={
            # Keep defaults unless you want to override
            'robot_name': 'tiago',
            'is_public_sim': 'True',
            'base_type': 'pmb2',
            'has_screen': 'False',
            'arm_type': 'tiago-arm',
            'arm_motor_model': 'parker',
            'end_effector': 'pal-gripper',
            'ft_sensor': 'schunk-ft',
            'wrist_model': 'wrist-2017',
            'camera_model': 'orbbec-astra',
            'laser_model': 'sick-571',
            'namespace': '',
        }.items()
    )

    return LaunchDescription([
        declare_world_name,
        # set_gazebo_resource_path,
        LogInfo(msg=['Launching Gazebo world file: ', world_file]),
        gazebo,
        LogInfo(msg=['Spawning TIAGo into Gazebo world']),
        TimerAction(
    		period=5.0,   # seconds delay
    		actions=[spawn_tiago]
	),
    ])

