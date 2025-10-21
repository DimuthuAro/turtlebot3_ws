import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
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
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_world_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('turtlebot3_maze'),
            'worlds',
            'maze_assignment.world'
        ]),
        description='Gazebo world file to load'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Launch Gazebo with world file and TurtleBot3
    gazebo_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ]),
        launch_arguments={
            'x_pose': '1.0',
            'y_pose': '1.0',
            'world': world_file,
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # SLAM Toolbox
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': PathJoinSubstitution([
                get_package_share_directory('turtlebot3_maze'),
                'config',
                'slam_params.yaml'
            ]),
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # RViz2 for visualization
    rviz_config = PathJoinSubstitution([
        get_package_share_directory('turtlebot3_maze'),
        'rviz',
        'slam.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        ld_preload_unset,
        declare_world_arg,
        declare_use_sim_time_arg,
        gazebo_turtlebot,
        slam_toolbox,
        rviz_node
    ])
