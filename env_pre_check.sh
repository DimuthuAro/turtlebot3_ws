#!/bin/bash

echo "════════════════════════════════════════════════════════════"
echo "  TurtleBot3 Maze - Environment Pre-Check & Setup"
echo "════════════════════════════════════════════════════════════"
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Track if any installations are needed
NEEDS_UPDATE=0
MISSING_DEPS=()

# Function to check if a package is installed
check_apt_package() {
    if dpkg -l | grep -q "^ii  $1 "; then
        echo -e "${GREEN}✓${NC} $1 is installed"
        return 0
    else
        echo -e "${RED}✗${NC} $1 is NOT installed"
        MISSING_DEPS+=("$1")
        NEEDS_UPDATE=1
        return 1
    fi
}

# Function to check if a ROS package exists
check_ros_package() {
    if ros2 pkg list 2>/dev/null | grep -q "^$1$"; then
        echo -e "${GREEN}✓${NC} ROS package $1 is installed"
        return 0
    else
        echo -e "${RED}✗${NC} ROS package $1 is NOT installed"
        MISSING_DEPS+=("$2")
        NEEDS_UPDATE=1
        return 1
    fi
}

# Function to check if a command exists
check_command() {
    if command -v $1 &> /dev/null; then
        echo -e "${GREEN}✓${NC} Command '$1' is available"
        return 0
    else
        echo -e "${RED}✗${NC} Command '$1' is NOT available"
        return 1
    fi
}

echo "════════════════════════════════════════════════════════════"
echo "  [1/5] Checking ROS 2 Jazzy Installation"
echo "════════════════════════════════════════════════════════════"
echo ""

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo -e "${GREEN}✓${NC} ROS 2 Jazzy is installed"
    source /opt/ros/jazzy/setup.bash
else
    echo -e "${RED}✗${NC} ROS 2 Jazzy is NOT installed!"
    echo ""
    echo "Please install ROS 2 Jazzy first:"
    echo "  https://docs.ros.org/en/jazzy/Installation.html"
    exit 1
fi

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  [2/5] Checking Gazebo (Ignition Gazebo)"
echo "════════════════════════════════════════════════════════════"
echo ""

if check_command gz; then
    GZ_VERSION=$(gz sim --versions 2>/dev/null | head -1)
    echo -e "${GREEN}✓${NC} Gazebo version: $GZ_VERSION"
else
    echo -e "${YELLOW}⚠${NC}  Gazebo command not found (should be in ROS 2 Jazzy)"
fi

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  [3/5] Checking Required ROS 2 Packages"
echo "════════════════════════════════════════════════════════════"
echo ""

check_ros_package "turtlebot3_gazebo" "ros-jazzy-turtlebot3-gazebo"
check_ros_package "turtlebot3_description" "ros-jazzy-turtlebot3"
check_ros_package "nav2_bringup" "ros-jazzy-navigation2"
check_ros_package "nav2_route" "ros-jazzy-nav2-route"
check_ros_package "slam_toolbox" "ros-jazzy-slam-toolbox"
check_ros_package "ros_gz_sim" "ros-jazzy-ros-gz-sim"
check_ros_package "ros_gz_bridge" "ros-jazzy-ros-gz-bridge"

echo ""
echo "Optional packages (not required for basic operation):"
if ros2 pkg list 2>/dev/null | grep -q "^gazebo_ros2_control$"; then
    echo -e "${GREEN}✓${NC} ROS package gazebo_ros2_control is installed"
else
    echo -e "${YELLOW}○${NC} ROS package gazebo_ros2_control is NOT installed (optional)"
fi

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  [4/5] Checking Python Dependencies"
echo "════════════════════════════════════════════════════════════"
echo ""

if python3 -c "import rclpy" 2>/dev/null; then
    echo -e "${GREEN}✓${NC} Python rclpy is available"
else
    echo -e "${RED}✗${NC} Python rclpy is NOT available"
    MISSING_DEPS+=("python3-rclpy")
    NEEDS_UPDATE=1
fi

if python3 -c "import numpy" 2>/dev/null; then
    echo -e "${GREEN}✓${NC} Python numpy is available"
else
    echo -e "${RED}✗${NC} Python numpy is NOT available"
    MISSING_DEPS+=("python3-numpy")
    NEEDS_UPDATE=1
fi

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  [5/5] Checking System Utilities"
echo "════════════════════════════════════════════════════════════"
echo ""

check_command "colcon"
check_command "rviz2"

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  Summary"
echo "════════════════════════════════════════════════════════════"
echo ""

if [ $NEEDS_UPDATE -eq 0 ]; then
    echo -e "${GREEN}✓ All dependencies are installed!${NC}"
    echo ""
    echo "You can now run:"
    echo "  ./run_automatic.sh"
    echo ""
    exit 0
else
    echo -e "${YELLOW}⚠ Missing dependencies detected!${NC}"
    echo ""
    echo "Missing packages:"
    for dep in "${MISSING_DEPS[@]}"; do
        echo "  - $dep"
    done
    echo ""
    
    read -p "Do you want to install missing dependencies now? (y/n): " -n 1 -r
    echo ""
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo ""
        echo "════════════════════════════════════════════════════════════"
        echo "  Installing Missing Dependencies"
        echo "════════════════════════════════════════════════════════════"
        echo ""
        
        # Update package list
        echo "Updating package list..."
        sudo apt update
        
        # Install missing packages
        if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
            echo ""
            echo "Installing: ${MISSING_DEPS[*]}"
            echo ""
            sudo apt install -y "${MISSING_DEPS[@]}"
            
            if [ $? -eq 0 ]; then
                echo ""
                echo -e "${GREEN}✓ All dependencies installed successfully!${NC}"
                echo ""
                echo "Please run this script again to verify the installation."
            else
                echo ""
                echo -e "${RED}✗ Some packages failed to install.${NC}"
                echo "Please check the errors above and install manually."
                exit 1
            fi
        fi
    else
        echo ""
        echo "Installation cancelled. Please install the missing dependencies manually:"
        echo ""
        echo "  sudo apt update"
        echo "  sudo apt install -y ${MISSING_DEPS[*]}"
        echo ""
        exit 1
    fi
fi

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  Workspace Check"
echo "════════════════════════════════════════════════════════════"
echo ""

if [ -f "install/setup.bash" ]; then
    echo -e "${GREEN}✓${NC} Workspace is built"
else
    echo -e "${YELLOW}⚠${NC}  Workspace not built yet"
    echo ""
    read -p "Do you want to build the workspace now? (y/n): " -n 1 -r
    echo ""
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo ""
        echo "Building workspace..."
        colcon build --packages-select turtlebot3_maze --symlink-install
        
        if [ $? -eq 0 ]; then
            echo ""
            echo -e "${GREEN}✓ Workspace built successfully!${NC}"
        else
            echo ""
            echo -e "${RED}✗ Workspace build failed.${NC}"
            exit 1
        fi
    fi
fi

echo ""
echo "════════════════════════════════════════════════════════════"
echo -e "  ${GREEN}✓ Environment Ready!${NC}"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "You can now run:"
echo "  export TURTLEBOT3_MODEL=burger"
echo "  ./run_automatic.sh"
echo ""
