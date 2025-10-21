#!/bin/bash

# TurtleBot3 Autonomous Navigation to Goal

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║        TurtleBot3 AUTONOMOUS Navigation to Goal               ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "🤖 Robot will automatically navigate to the goal!"
echo ""
echo "📍 Start Position: (0.5, 0.5)"
echo "🎯 Goal Position: (5.0, 5.0) - Green Sphere"
echo ""
echo "🔄 Navigation Strategy:"
echo "  - Uses LiDAR to detect walls"
echo "  - Automatically avoids obstacles"
echo "  - Turns toward goal direction"
echo "  - Moves forward when path is clear"
echo ""
echo "════════════════════════════════════════════════════════════════"
echo ""

# Set environment
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

# Source ROS
source /opt/ros/humble/setup.bash
cd ~/turtlebot3_ws
source install/setup.bash

echo "🚀 Launching Gazebo with Square Maze..."

# Launch Gazebo in background
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py \
    world:=$HOME/turtlebot3_ws/src/turtlebot3_maze/worlds/square_maze.world \
    x_pose:=0.5 \
    y_pose:=0.5 &

GAZEBO_PID=$!

echo "⏳ Waiting for Gazebo to initialize (10 seconds)..."
sleep 10

echo ""
echo "🤖 Starting AUTONOMOUS NAVIGATION..."
echo ""
echo "Watch the robot navigate to the green sphere!"
echo "Press Ctrl+C to stop"
echo ""
echo "════════════════════════════════════════════════════════════════"
echo ""

# Start autonomous navigator
ros2 run turtlebot3_maze simple_auto_navigator

# Cleanup
kill $GAZEBO_PID 2>/dev/null
