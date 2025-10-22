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
    
    # World file - use Pacman-style maze
    world_file = PathJoinSubstitution([
        turtlebot3_maze_dir,
        'worlds',
        'maze_assignment.world'
    ])
    
    # 1. Launch Gazebo (use ros_gz_sim directly so we can pass our custom world)
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

    # Spawn TurtleBot3 using the package's spawn launch
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

    # Robot state publisher (reuse turtlebot3 spawn's publisher or include separately if needed)
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
        period=20.0,  # Wait longer for Nav2 to be ready (increased from 12s)
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
        gzserver_cmd,
        gzclient_cmd,  # Re-enabled: Gazebo GUI
        spawn_turtlebot,
        robot_state_publisher,
        slam_toolbox,
        nav2_bringup,
        rviz_node,  # Re-enabled: RViz visualization
        goal_navigator  # Re-enabled: Autonomous goal navigator
    ])
