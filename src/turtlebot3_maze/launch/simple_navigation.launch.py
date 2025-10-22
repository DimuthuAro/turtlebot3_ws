#!/usr/bin/env python3
"""
Simple navigation launch file - just Gazebo and simple autonomous navigator
No SLAM, no Nav2 - just basic autonomous movement with obstacle avoidance
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Fix for snap library conflicts
    ld_preload_unset = SetEnvironmentVariable(
        name='LD_PRELOAD',
        value=''
    )
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Get package directories
    turtlebot3_maze_dir = get_package_share_directory('turtlebot3_maze')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # World file - use Pacman-style maze
    world_file = PathJoinSubstitution([
        turtlebot3_maze_dir,
        'worlds',
        'maze_assignment.world'
    ])
    
    # 1. Launch Gazebo
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # gzserver (headless) with our world
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                ros_gz_sim_dir,
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # gzclient (GUI)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                ros_gz_sim_dir,
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': '-g -v2 ',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # 2. Spawn TurtleBot3
    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                turtlebot3_gazebo_dir,
                'launch',
                'spawn_turtlebot3.launch.py'
            ])
        ]),
        launch_arguments={
            'x_pose': '0.5',
            'y_pose': '0.5',
            'z_pose': '0.05'
        }.items()
    )

    # 3. Robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                turtlebot3_gazebo_dir,
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # 4. Simple autonomous navigator (delayed start to allow Gazebo to be ready)
    simple_navigator = TimerAction(
        period=8.0,  # Wait for Gazebo to be ready
        actions=[
            Node(
                package='turtlebot3_maze',
                executable='simple_auto_navigator.py',  # Add .py extension
                name='simple_auto_navigator',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    return LaunchDescription([
        ld_preload_unset,
        declare_use_sim_time_arg,
        gzserver_cmd,
        gzclient_cmd,
        spawn_turtlebot,
        robot_state_publisher,
        simple_navigator
    ])
