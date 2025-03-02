from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    #generate Launch description
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    sdf_robot_file = LaunchConfiguration('sdf_robot_file')

    return LaunchDescription([
        # Declare Launch descrption
        DeclareLaunchArgument('robot_name', default_value='', description='Name of the robot'),
        DeclareLaunchArgument('x', default_value='', description='X position'),
        DeclareLaunchArgument('y', default_value='', description='Y position'),
        DeclareLaunchArgument('z', default_value='', description='Z position'),
        DeclareLaunchArgument('roll', default_value='', description='Roll orientation'),
        DeclareLaunchArgument('pitch', default_value='', description='Pitch orientation'),
        DeclareLaunchArgument('yaw', default_value='', description='Yaw orientation'),
        DeclareLaunchArgument('sdf_robot_file', default_value='', description='Path to the SDF file'),

        # call Gazebo spawn_entity 
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=[robot_name, '_spawn_entity'],
            output='screen',
            arguments=[
                '-entity', robot_name,
                '-file', sdf_robot_file,
                '-x', x,
                '-y', y,
                '-z', z,
                '-R', roll,
                '-P', pitch,
                '-Y', yaw
            ]
        )
    ])