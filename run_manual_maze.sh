#!/bin/bash

# Simple Manual Navigation in Square Maze
# You can drive the robot manually to the green goal using keyboard

echo "=============================================="
echo "  TurtleBot3 Square Maze - Manual Control"
echo "=============================================="
echo ""
echo "Starting system in 3 terminals..."
echo ""

# Set environment
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

# Source ROS
source /opt/ros/humble/setup.bash
cd ~/turtlebot3_ws
source install/setup.bash

echo "📍 Starting Position: (0.5, 0.5)"
echo "🎯 Goal Position: (5.0, 5.0) - Look for GREEN SPHERE"
echo ""
echo "Opening Gazebo with square maze..."

# Launch Gazebo with square maze in background
gazebo --verbose ~/turtlebot3_ws/src/turtlebot3_maze/worlds/square_maze.world &
GAZEBO_PID=$!

# Wait for Gazebo to start
sleep 8

echo ""
echo "Spawning TurtleBot3 robot..."
ros2 run gazebo_ros spawn_entity.py -entity turtlebot3 \
    -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
    -x 0.5 -y 0.5 -z 0.05 &

sleep 3

echo ""
echo "✅ System ready!"
echo ""
echo "=============================================="
echo "  MANUAL CONTROL INSTRUCTIONS"
echo "=============================================="
echo ""
echo "In Gazebo window:"
echo "  - You should see a 6x6 meter square maze"
echo "  - BLUE walls = boundaries"
echo "  - RED walls = internal maze obstacles"
echo "  - GREEN SPHERE at top-right = GOAL (5, 5)"
echo "  - TurtleBot3 at bottom-left = START (0.5, 0.5)"
echo ""
echo "To control the robot, run in a NEW terminal:"
echo "  export TURTLEBOT3_MODEL=burger"
echo "  ros2 run turtlebot3_teleop teleop_keyboard"
echo ""
echo "Keyboard controls:"
echo "  i = forward"
echo "  j = turn left"
echo "  k = stop"
echo "  l = turn right"
echo "  , = backward"
echo ""
echo "Navigate the robot through the maze to reach"
echo "the GREEN SPHERE goal!"
echo ""
echo "Press Ctrl+C here to stop Gazebo"
echo "=============================================="

# Wait for user to stop
wait $GAZEBO_PID
