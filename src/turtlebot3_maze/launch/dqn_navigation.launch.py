import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
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
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    model_path = LaunchConfiguration('model_path')
    
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
    
    declare_model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution([
            get_package_share_directory('turtlebot3_maze'),
            'models',
            'dqn_trained.pth'
        ]),
        description='Path to pre-trained DQN model'
    )
    
    # Launch Gazebo with world file
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )
    
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    # Spawn TurtleBot3
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'spawn_turtlebot3.launch.py'
            ])
        ]),
        launch_arguments={
            'x_pose': '1.0',
            'y_pose': '1.0'
        }.items()
    )
    
    # DQN Navigation Node
    dqn_navigation_node = Node(
        package='turtlebot3_maze',
        executable='dqn_navigation',
        name='dqn_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': model_path,
            'state_size': 27,  # 24 lidar + 2 relative goal + 1 distance
            'action_size': 5,  # Forward, Backward, Left, Right, Stop
            'learning_rate': 0.001,
            'gamma': 0.99
        }]
    )
    
    # RViz2 for visualization
    rviz_config = PathJoinSubstitution([
        get_package_share_directory('turtlebot3_maze'),
        'rviz',
        'slam.rviz'  # Using SLAM config since we don't need navigation displays
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
        declare_model_path_arg,
        gazebo_server,
        gazebo_client,
        spawn_entity,
        dqn_navigation_node,
        rviz_node
    ])
