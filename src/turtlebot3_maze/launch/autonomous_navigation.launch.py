#!/usr/bin/env python3
"""
Complete autonomous navigation launch file
Launches Gazebo, SLAM, Nav2, and goal navigator
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
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # World file
    world_file = PathJoinSubstitution([
        turtlebot3_maze_dir,
        'worlds',
        'square_maze.world'
    ])
    
    # 1. Launch Gazebo with TurtleBot3
    gazebo_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                turtlebot3_gazebo_dir,
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ]),
        launch_arguments={
            'x_pose': '0.5',
            'y_pose': '0.5',
            'z_pose': '0.05',
            'world': world_file,
            'use_sim_time': 'true'
        }.items()
    )
    
    # 2. Launch SLAM Toolbox
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                slam_toolbox_dir,
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': PathJoinSubstitution([
                turtlebot3_maze_dir,
                'config',
                'slam_params.yaml'
            ]),
            'use_sim_time': 'true'
        }.items()
    )
    
    # 3. Launch Nav2
    nav2_params_file = PathJoinSubstitution([
        turtlebot3_maze_dir,
        'config',
        'nav2_params.yaml'
    ])
    
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                nav2_bringup_dir,
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'true'
        }.items()
    )
    
    # 4. RViz for visualization
    rviz_config = PathJoinSubstitution([
        turtlebot3_maze_dir,
        'rviz',
        'navigation.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # 5. Goal Navigator (delayed start to allow Nav2 to initialize)
    goal_navigator = TimerAction(
        period=10.0,  # Wait 10 seconds for Nav2 to be ready
        actions=[
            Node(
                package='turtlebot3_maze',
                executable='goal_navigator',
                name='goal_navigator',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    return LaunchDescription([
        ld_preload_unset,
        declare_use_sim_time_arg,
        gazebo_turtlebot,
        slam_toolbox,
        nav2_bringup,
        rviz_node,
        goal_navigator
    ])
