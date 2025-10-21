#!/bin/bash

# TurtleBot3 Square Maze - Simple Launch Script

echo "=============================================="
echo "  TurtleBot3 Square Maze with Goal Marker"
echo "=============================================="
echo ""
echo "🎯 Goal: Navigate to the GREEN sphere at (5, 5)"
echo "📍 Start: Robot spawns at (0.5, 0.5)"
echo ""

# Set environment
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

# Source ROS
source /opt/ros/humble/setup.bash
cd ~/turtlebot3_ws
source install/setup.bash

echo "🚀 Launching Gazebo with Square Maze..."
echo ""
echo "After Gazebo opens:"
echo "  1. Look for the GREEN SPHERE (goal) at position (5, 5)"
echo "  2. TurtleBot3 will spawn at (0.5, 0.5)"
echo "  3. Navigate through the maze to reach the green sphere"
echo ""
echo "To control the robot, open a NEW terminal and run:"
echo "  export TURTLEBOT3_MODEL=burger"
echo "  source /opt/ros/humble/setup.bash"
echo "  ros2 run turtlebot3_teleop teleop_keyboard"
echo ""
echo "=============================================="
echo ""

# Use the official turtlebot3_gazebo launch with our world
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py \
    world:=$HOME/turtlebot3_ws/src/turtlebot3_maze/worlds/square_maze.world \
    x_pose:=0.5 \
    y_pose:=0.5
