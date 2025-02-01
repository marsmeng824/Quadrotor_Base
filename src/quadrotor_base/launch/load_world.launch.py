import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 自定义世界文件路径
    world_file = '/home/mars0824/ros2_ws/src/quadrotor_base/worlds/world_ini.sdf'

    # 启动 Gazebo 并加载世界文件
    gazebo = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    return LaunchDescription([gazebo, gzclient])
