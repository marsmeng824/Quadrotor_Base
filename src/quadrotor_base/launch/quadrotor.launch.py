#!/usr/bin/python3
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 启动 Gazebo 仿真
    gazebo = ExecuteProcess(
        cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py'],
        output='screen'
    )

    # 启动自定义的 quadrotor 节点
    quadrotor_node = Node(
        package='quadrotor_base',  # 替换为包含 quadrotor_spawner 的包名
        executable='quadrotor_spawner',  # 这是你自定义的节点
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        quadrotor_node,  # 确保这个节点会启动并生成模型
    ])
