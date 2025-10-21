#!/bin/bash

echo "================================================"
echo "TurtleBot3 Project - Final Verification Script"
echo "================================================"
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Source workspace
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/turtlebot3_ws/install/setup.bash 2>/dev/null
export TURTLEBOT3_MODEL=burger

echo "Checking project structure..."
echo "================================================"

check_file() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✓${NC} $1"
        return 0
    else
        echo -e "${RED}✗${NC} $1 (MISSING)"
        return 1
    fi
}

check_dir() {
    if [ -d "$1" ]; then
        echo -e "${GREEN}✓${NC} $1/"
        return 0
    else
        echo -e "${RED}✗${NC} $1/ (MISSING)"
        return 1
    fi
}

# Check launch files
echo ""
echo "Launch Files:"
check_file "~/turtlebot3_ws/src/turtlebot3_maze/launch/slam.launch.py"
check_file "~/turtlebot3_ws/src/turtlebot3_maze/launch/navigation.launch.py"
check_file "~/turtlebot3_ws/src/turtlebot3_maze/launch/dqn_navigation.launch.py"

# Check world files
echo ""
echo "World Files:"
check_file "~/turtlebot3_ws/src/turtlebot3_maze/worlds/maze_assignment.world"
check_file "~/turtlebot3_ws/src/turtlebot3_maze/worlds/maze_simple.world"
check_file "~/turtlebot3_ws/src/turtlebot3_maze/worlds/maze_complex.world"

# Check Python modules
echo ""
echo "Python Modules:"
check_file "~/turtlebot3_ws/src/turtlebot3_maze/turtlebot3_maze/dqn_agent.py"
check_file "~/turtlebot3_ws/src/turtlebot3_maze/turtlebot3_maze/dqn_navigation.py"
check_file "~/turtlebot3_ws/src/turtlebot3_maze/turtlebot3_maze/maze_generator.py"

# Check config files
echo ""
echo "Configuration Files:"
check_file "~/turtlebot3_ws/src/turtlebot3_maze/config/nav2_params.yaml"
check_file "~/turtlebot3_ws/src/turtlebot3_maze/config/slam_params.yaml"
check_file "~/turtlebot3_ws/src/turtlebot3_maze/config/dqn_config.yaml"

# Check RViz configs
echo ""
echo "RViz Configurations:"
check_file "~/turtlebot3_ws/src/turtlebot3_maze/rviz/slam.rviz"
check_file "~/turtlebot3_ws/src/turtlebot3_maze/rviz/navigation.rviz"

# Check directories
echo ""
echo "Required Directories:"
check_dir "~/turtlebot3_ws/src/turtlebot3_maze/maps"
check_dir "~/turtlebot3_ws/src/turtlebot3_maze/models"

# Check build
echo ""
echo "Build Status:"
if [ -d "~/turtlebot3_ws/install/turtlebot3_maze" ]; then
    echo -e "${GREEN}✓${NC} Package installed"
else
    echo -e "${RED}✗${NC} Package not installed"
fi

# Check ROS packages
echo ""
echo "ROS 2 Dependencies:"
echo "================================================"

check_ros_pkg() {
    if ros2 pkg list | grep -q "^$1$"; then
        echo -e "${GREEN}✓${NC} $1"
        return 0
    else
        echo -e "${RED}✗${NC} $1 (not installed)"
        return 1
    fi
}

check_ros_pkg "turtlebot3_gazebo"
check_ros_pkg "slam_toolbox"
check_ros_pkg "nav2_bringup"
check_ros_pkg "teleop_twist_keyboard"

echo ""
echo "================================================"
echo "Verification complete!"
echo "================================================"
echo ""
echo "To run the project:"
echo "  SLAM:       ros2 launch turtlebot3_maze slam.launch.py"
echo "  Navigation: ros2 launch turtlebot3_maze navigation.launch.py"
echo "  DQN:        ros2 launch turtlebot3_maze dqn_navigation.launch.py"
echo ""
echo "Or use the helper script:"
echo "  ./run_humble.sh"
echo ""
