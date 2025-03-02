from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # define Launch arguments
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    sdf_robot_file = LaunchConfiguration('sdf_robot_file')

    return LaunchDescription([
        # declare Launch arguments
        DeclareLaunchArgument('robot_name', default_value='F450', description='Name of the robot'),
        DeclareLaunchArgument('x', default_value='0.0', description='X position'),
        DeclareLaunchArgument('y', default_value='0.0', description='Y position'),
        DeclareLaunchArgument('z', default_value='0.0', description='Z position'),
        DeclareLaunchArgument('roll', default_value='0.0', description='Roll orientation'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='Pitch orientation'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw orientation'),
        DeclareLaunchArgument('sdf_robot_file', default_value=PathJoinSubstitution([
            FindPackageShare('f450_simulation'), 'models','F450', 'model.sdf'
        ]), description='Path to the SDF file'),

        # launch gazebo world
        ExecuteProcess(
            cmd=['gazebo', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen' ),

        #  spawn_F450 Launch 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('f450_simulation'), 'launch', 'spawn_F450.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_name': robot_name,
                'x': x,
                'y': y,
                'z': z,
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw,
                'sdf_robot_file': sdf_robot_file
            }.items()
        )
    ])
          
