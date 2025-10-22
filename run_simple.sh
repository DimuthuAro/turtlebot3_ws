#!/bin/bash

echo "════════════════════════════════════════════════════════════"
echo "  TurtleBot3 Simple Navigation Launcher"
echo "════════════════════════════════════════════════════════════"
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Kill all relevant processes first
echo -e "${YELLOW}[0/3] Cleaning up previous processes...${NC}"

# Kill Gazebo processes
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
pkill -9 gazebo 2>/dev/null

# Kill ROS 2 nodes
pkill -9 -f "ros2" 2>/dev/null
pkill -9 -f "robot_state_publisher" 2>/dev/null
pkill -9 -f "simple_auto_navigator" 2>/dev/null

# Wait a moment for processes to terminate
sleep 2

echo -e "${GREEN}✓ Previous processes cleaned up${NC}"
echo ""

# Change to workspace directory
cd ~/turtlebot3_ws

# Source ROS and workspace
echo -e "${GREEN}[1/3] Sourcing ROS 2 and workspace...${NC}"

# Fix Gazebo rendering issues for VM
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330
export SVGA_VGPU10=0
export LIBGL_ALWAYS_INDIRECT=0

# Gazebo performance settings
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/install/turtlebot3_maze/share/turtlebot3_maze/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/turtlebot3_ws/install/turtlebot3_maze/share/turtlebot3_maze

export GAZEBO_GUI_SKIP_RENDERING=0

source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Export TurtleBot3 model
echo -e "${GREEN}[2/3] Setting TurtleBot3 model to burger...${NC}"
export TURTLEBOT3_MODEL=burger

# Launch simple navigation
echo -e "${GREEN}[3/3] Launching Gazebo with Simple Navigation...${NC}"
echo -e "${YELLOW}      (This will take 10-15 seconds to load)${NC}"
ros2 launch turtlebot3_maze simple_navigation.launch.py > /tmp/turtlebot3_simple.log 2>&1 &
LAUNCH_PID=$!

# Wait for Gazebo to fully start
echo ""
echo "⏳ Waiting for Gazebo to initialize..."
sleep 12

# Check if launch is still running
if ! ps -p $LAUNCH_PID > /dev/null 2>&1; then
    echo -e "\n❌ Error: Launch failed to start!"
    echo "Check log: /tmp/turtlebot3_simple.log"
    tail -n 50 /tmp/turtlebot3_simple.log
    exit 1
fi

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  ✅ SIMPLE NAVIGATION READY!"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "📺 WHAT TO EXPECT:"
echo "  • Gazebo window with maze and TurtleBot3"
echo "  • Robot will start moving autonomously after ~8 seconds"
echo "  • Robot navigates to goal at (5.0, 5.0) using LiDAR"
echo "  • Avoids obstacles automatically"
echo ""
echo "🤖 FEATURES:"
echo "  • No SLAM or Nav2 required"
echo "  • Direct LiDAR-based obstacle avoidance"
echo "  • Simple but effective autonomous movement"
echo ""
echo "⚠️  NOTE:"
echo "  • If Gazebo shows black screen, it's a VM graphics issue"
echo "  • Robot will still navigate - check terminal for progress"
echo ""
echo "🛑 TO STOP:"
echo "  Press Ctrl+C to quit everything"
echo ""
echo "════════════════════════════════════════════════════════════"
echo ""

# Wait for user to stop (keep the script alive while launch runs)
trap 'echo "\n🛑 Shutting down..."; kill $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null; echo "✅ Simulation stopped."; exit 0' SIGINT SIGTERM

while kill -0 $LAUNCH_PID 2>/dev/null; do
    sleep 1
done

# If the launch process exits on its own, report and show last log lines
echo "\n⚠️ Launch exited. See /tmp/turtlebot3_simple.log for details"
tail -n 40 /tmp/turtlebot3_simple.log || true
