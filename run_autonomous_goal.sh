#!/bin/bash

# TurtleBot3 Autonomous Goal Navigation Script
# This script launches the complete autonomous navigation system

echo "=============================================="
echo "  TurtleBot3 Autonomous Goal Navigator"
echo "=============================================="
echo ""
echo "🤖 System Components:"
echo "  ✓ Square Maze World (6x6 meters)"
echo "  ✓ TurtleBot3 Burger Robot"
echo "  ✓ SLAM Toolbox (Real-time mapping)"
echo "  ✓ Nav2 (Path planning & navigation)"
echo "  ✓ Goal Navigator (Autonomous controller)"
echo ""
echo "📍 Starting Position: (0.5, 0.5)"
echo "🎯 Goal Position: (5.0, 5.0) - Green Sphere"
echo ""
echo "⏳ The robot will:"
echo "  1. Start at position (0.5, 0.5)"
echo "  2. Build a map using LiDAR and SLAM"
echo "  3. Automatically navigate to the green goal marker"
echo "  4. Use LiDAR to detect and avoid obstacles"
echo ""
echo "=============================================="
echo ""

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Source workspace
cd ~/turtlebot3_ws
source install/setup.bash

echo "🚀 Launching autonomous navigation system..."
echo ""
echo "Windows will open:"
echo "  - Gazebo: 3D simulation with square maze"
echo "  - RViz: Visualization of map and navigation"
echo ""
echo "Wait ~10 seconds after launch, then the robot will"
echo "automatically start navigating to the goal!"
echo ""
echo "Press Ctrl+C to stop the simulation"
echo ""
echo "=============================================="

# Launch the autonomous navigation
ros2 launch turtlebot3_maze autonomous_navigation.launch.py
