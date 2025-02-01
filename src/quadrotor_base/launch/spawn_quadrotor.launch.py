#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # 设置模型路径
    model_path = '/home/mars0824/ros2_ws/src/quadrotor_base/models/quadrotor/model.sdf'
    


    # 启动 Gazebo 仿真
    gazebo = ExecuteProcess(
        cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py'],
        output='screen'
    )

    # 等待 Gazebo 完成启动，然后加载自定义模型
    spawn_entity = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', 
             '-entity', 'quadrotor', '-file', model_path],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
    ])
