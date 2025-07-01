#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the package directory
    pkg_path = get_package_share_directory('arm6')
    xacro_file = os.path.join(pkg_path, 'urdf', 'arm6.urdf.xacro')
    controllers_file = os.path.join(pkg_path, 'config', 'arm_controllers.yaml')
    world_file = os.path.join(pkg_path, 'worlds', 'empty.world')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default='false')
    world = LaunchConfiguration('world', default=world_file)
    
    # Robot description using XACRO with absolute path to controllers file
    robot_description_content = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' controllers_file:=', controllers_file,
            ' use_sim_time:=', use_sim_time,
            ' use_fake_hardware:=', use_fake_hardware
        ]),
        value_type=str
    )
    
    # Gazebo launch
    gazebo_launch = None
    try:
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('gazebo_ros'),
                '/launch/gazebo.launch.py'
            ]),
            launch_arguments={
                'world': world,
                'verbose': 'true'
            }.items()
        )
    except Exception as e:
        print(f"Warning: Could not include Gazebo launch: {e}")
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'arm6',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # Joint trajectory controller
    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )
    
    # Build launch description
    launch_nodes = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware interface if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Full path to world model file to load'
        ),
        robot_state_publisher,
        joint_state_broadcaster,
        joint_trajectory_controller
    ]
    
    # Add Gazebo if available
    if gazebo_launch is not None:
        launch_nodes.extend([gazebo_launch, spawn_entity])
    else:
        print("Warning: Gazebo launch not available, robot will be published without simulation")
    
    return LaunchDescription(launch_nodes)
