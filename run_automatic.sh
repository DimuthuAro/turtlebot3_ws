#!/bin/bash

echo "════════════════════════════════════════════════════════════"
echo "  TurtleBot3 Maze Navigation - Automatic Launcher"
echo "════════════════════════════════════════════════════════════"
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Change to workspace directory
cd ~/turtlebot3_ws

# Source ROS and workspace
echo -e "${GREEN}[1/4] Sourcing ROS 2 Humble...${NC}"
source /opt/ros/humble/setup.bash
source install/setup.bash

# Export TurtleBot3 model
echo -e "${GREEN}[2/4] Setting TurtleBot3 model to burger...${NC}"
export TURTLEBOT3_MODEL=burger

# Launch SLAM in background
echo -e "${GREEN}[3/4] Launching Gazebo + SLAM + RViz...${NC}"
echo -e "${YELLOW}      (This will take 20-30 seconds to load)${NC}"
ros2 launch turtlebot3_maze slam.launch.py > /tmp/turtlebot3_slam.log 2>&1 &
SLAM_PID=$!

# Wait for Gazebo to fully start
echo ""
echo "⏳ Waiting for Gazebo and RViz to initialize..."
sleep 25

# Check if SLAM is still running
if ! ps -p $SLAM_PID > /dev/null; then
    echo -e "\n❌ Error: SLAM failed to start!"
    echo "Check log: /tmp/turtlebot3_slam.log"
    exit 1
fi

# Launch teleop control
echo -e "\n${GREEN}[4/4] Launching keyboard control...${NC}"
echo ""
echo "════════════════════════════════════════════════════════════"
echo "  ✅ SIMULATION READY!"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "🎮 KEYBOARD CONTROLS:"
echo "────────────────────────────────────────────────────────────"
echo "    i = Forward        k = Stop"
echo "    , = Backward       j = Left"
echo "                       l = Right"
echo ""
echo "    q = Faster         z = Slower"
echo "────────────────────────────────────────────────────────────"
echo ""
echo "📺 YOU SHOULD SEE:"
echo "  • Gazebo window (3D maze with robot)"
echo "  • RViz window (map being built)"
echo ""
echo "🚀 START DRIVING:"
echo "  Press 'i' to move forward and explore the maze!"
echo ""
echo "🛑 TO STOP:"
echo "  Press Ctrl+C to quit everything"
echo ""
echo "════════════════════════════════════════════════════════════"
echo ""

# Source ROS for teleop
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

# Run teleop (this keeps the script running)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Cleanup when teleop exits
echo ""
echo "🛑 Shutting down simulation..."
kill $SLAM_PID 2>/dev/null
wait $SLAM_PID 2>/dev/null
echo "✅ Simulation stopped."
