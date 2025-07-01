#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 Launch文件用于显示arm_urdf机器人模型
适配自原始的ROS1 launch文件
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('arm_urdf')
    
    # URDF文件路径
    urdf_file = os.path.join(pkg_share, 'urdf', 'arm2.0.urdf')
    
    # 声明launch参数
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=urdf_file,
        description='URDF文件的绝对路径'
    )
    
    # 读取URDF文件内容
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Robot State Publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    # Joint State Publisher GUI节点
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'arm_urdf.rviz')]
    )
    
    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])