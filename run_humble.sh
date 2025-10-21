#!/bin/bash

echo "================================================"
echo "TurtleBot3 SLAM - ROS 2 Humble Version"
echo "================================================"

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash

# Set TurtleBot model
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

echo "✓ Environment configured for ROS 2 Humble"
echo "✓ TurtleBot3 model: burger"
echo ""
echo "Starting SLAM system..."
echo "================================================"

# Launch the simulation
ros2 launch turtlebot3_maze slam.launch.py "$@"
