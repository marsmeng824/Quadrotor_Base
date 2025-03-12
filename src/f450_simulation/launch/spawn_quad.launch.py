import os 
from launch import LaunchDescription 
from launch.actions  import ExecuteProcess,DeclareLaunchArgument, IncludeLaunchDescription 
from launch.substitutions  import LaunchConfiguration, PathJoinSubstitution 
from launch_ros.actions  import Node 
from launch_ros.substitutions  import FindPackageShare 
from ament_index_python.packages  import get_package_share_directory 
 
def generate_launch_description():
    # Get the shared directory of the current package
    world_path = PathJoinSubstitution([
    FindPackageShare('f450_simulation'),
    'world',
    'world_set.sdf' ])
    
    # switch on Gazebo 
    gazebo_node =ExecuteProcess(cmd=['gazebo',world_path,' --verbose',
            '-s', 'libgazebo_ros_init.so', 
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_force_system.so',
            ],
            output='screen' )

    model_path = PathJoinSubstitution([
        FindPackageShare('f450_simulation'),
        'models',
        'F450',
        'model.sdf'])

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py', 
        arguments=[
            '-entity', 'F450',
            '-file', model_path,  # find the path
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
            '-timeout', '3'
        ],
       output='screen'
    )

    
    return LaunchDescription([
        gazebo_node,
        spawn_entity_node 
    ])