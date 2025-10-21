#!/bin/bash

# TurtleBot3 Autonomous Navigation to Goal

echo "=============================================="
echo "  🤖 TurtleBot3 AUTONOMOUS NAVIGATION"
echo "=============================================="
echo ""
echo "Starting autonomous navigation system..."
echo ""
echo "📍 Start: (0.5, 0.5)"
echo "🎯 Goal: (5.0, 5.0) - Green Sphere"
echo ""
echo "The robot will:"
echo "  ✓ Use LiDAR to detect obstacles"
echo "  ✓ Navigate around walls automatically"
echo "  ✓ Drive to the green goal sphere"
echo ""
echo "=============================================="

# Set environment
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

# Source ROS
source /opt/ros/humble/setup.bash
cd ~/turtlebot3_ws
source install/setup.bash

echo ""
echo "🚀 Launching Gazebo..."

# Launch Gazebo with square maze in background
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py \
    world:=$HOME/turtlebot3_ws/src/turtlebot3_maze/worlds/square_maze.world \
    x_pose:=0.5 \
    y_pose:=0.5 &

GAZEBO_PID=$!

# Wait for Gazebo to initialize
sleep 8

echo ""
echo "🤖 Starting autonomous navigator..."
echo ""
echo "Watch the robot navigate to the green sphere!"
echo ""
echo "Press Ctrl+C to stop"
echo "=============================================="
echo ""

# Run the autonomous navigator
ros2 run turtlebot3_maze simple_auto_navigator

# Cleanup
kill $GAZEBO_PID 2>/dev/null
