# TurtleBot3 Navigation - Quick Start Guide

## 🤖 Make the TurtleBot Move Around!

Your TurtleBot3 is now ready to move autonomously! Here are the simple steps:

### Option 1: Simple Autonomous Navigation (RECOMMENDED - Easiest!)

This uses basic LiDAR-based obstacle avoidance - no complex SLAM or Nav2 needed!

```bash
cd ~/turtlebot3_ws
./run_simple.sh
```

**What happens:**
- Gazebo opens with your maze
- TurtleBot3 spawns at position (0.5, 0.5)
- After 8 seconds, the robot starts moving toward goal (5.0, 5.0)
- Robot uses LiDAR to detect and avoid obstacles
- Robot will navigate autonomously until it reaches the goal!

**Features:**
- ✅ Direct LiDAR control
- ✅ Automatic obstacle avoidance  
- ✅ No complex setup needed
- ✅ Works even on VMs with graphics issues

**To stop:** Press `Ctrl+C` in the terminal

---

### Option 2: Full Autonomous Navigation (Advanced)

This uses SLAM + Nav2 for advanced navigation with mapping:

```bash
cd ~/turtlebot3_ws
./run_automatic.sh
```

**What happens:**
- Gazebo opens with your maze
- RViz opens showing real-time map building
- TurtleBot3 uses SLAM to create a map while navigating
- Nav2 handles path planning and obstacle avoidance
- After ~20 seconds, robot starts autonomous navigation

**Features:**
- ✅ Real-time SLAM mapping
- ✅ Advanced path planning
- ✅ RViz visualization
- ✅ Production-ready navigation stack

**To stop:** Press `Ctrl+C` in the terminal

---

### Option 3: Manual Control (Teleoperation)

Control the robot manually with your keyboard:

```bash
# Terminal 1 - Launch Gazebo
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2 - Teleop control
source /opt/ros/jazzy/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Controls:**
- `w` - Move forward
- `x` - Move backward
- `a` - Turn left
- `d` - Turn right
- `s` - Stop
- `Space` - Emergency stop

---

## 📊 What's Happening

### Simple Navigation (`./run_simple.sh`)
```
Robot → LiDAR Sensors → Obstacle Detection → Movement Commands → Motors
         360° scan       <40cm = obstacle     Linear + Angular    Wheels turn
```

### Full Navigation (`./run_automatic.sh`)
```
Robot → SLAM → Map → Nav2 Path Planner → Controller → Motors
      Build map  Save   Find best path     Follow path   Move
```

---

## 🎯 Goal Position

By default, the robot navigates to position **(5.0, 5.0)** in the maze.

To change the goal, edit the Python files:
- Simple: `src/turtlebot3_maze/turtlebot3_maze/simple_auto_navigator.py` (lines 21-22)
- Full: `src/turtlebot3_maze/turtlebot3_maze/goal_navigator.py` (lines 22-23)

Then rebuild:
```bash
colcon build --packages-select turtlebot3_maze
```

---

## 🔧 Troubleshooting

### Gazebo shows black screen
- This is a VM graphics issue
- The robot IS still navigating!
- Check the terminal output to see progress
- The simple navigator prints distance updates

### Robot doesn't move
1. Wait 8-10 seconds after "Waiting for Gazebo to initialize..."
2. Check if you see log messages about distance to goal
3. Try restarting: `Ctrl+C` then run the script again

### Launch fails immediately
1. Clean previous builds:
   ```bash
   rm -rf build/ install/ log/
   colcon build --packages-select turtlebot3_maze
   ```
2. Try running again

### "Process has died" error
- Check `/tmp/turtlebot3_simple.log` or `/tmp/turtlebot3_autonomous.log`
- Usually means Nav2 isn't ready yet (for autonomous mode)
- Simple mode should work without issues

---

## 📁 Files Overview

### Launch Files
- `simple_navigation.launch.py` - Basic autonomous navigation
- `autonomous_navigation.launch.py` - Full SLAM + Nav2 navigation
- `slam.launch.py` - SLAM mapping only

### Python Scripts
- `simple_auto_navigator.py` - Direct LiDAR control for navigation
- `goal_navigator.py` - Nav2-based goal reaching
- `maze_gui.py` - Maze editor GUI

### Scripts
- `run_simple.sh` - Quick start for basic navigation ⭐
- `run_automatic.sh` - Full autonomous system
- `run.sh` - Alternative launcher

---

## 🚀 Quick Commands

```bash
# Start simple navigation (EASIEST!)
./run_simple.sh

# Start full autonomous navigation
./run_automatic.sh

# Stop everything
Ctrl+C

# View robot position
ros2 topic echo /odom

# View LiDAR data
ros2 topic echo /scan

# List all active ROS nodes
ros2 node list

# Check robot status
ros2 topic list
```

---

## ✅ Success Indicators

You know it's working when you see:

### Simple Navigation
```
🤖 Simple Auto Navigator Started!
📍 Current: (0.50, 0.50)
🎯 Goal: (5.00, 5.00)
📍 Distance to goal: 6.36m | Front: 3.45m | Left: 2.10m | Right: 1.85m
📍 Distance to goal: 5.82m | Front: 3.01m | Left: 2.34m | Right: 1.92m
...
🎉🎉🎉 GOAL REACHED! 🎉🎉🎉
```

### Full Navigation
```
✅ AUTONOMOUS SIMULATION READY!
[goal_navigator]: Nav2 action server ready!
[goal_navigator]: Sending goal to Nav2: (5.00, 5.00)
[goal_navigator]: Goal accepted by Nav2! Robot is navigating...
[goal_navigator]: Distance to goal: 4.23m
...
🎉 SUCCESS! Robot reached the goal! 🎉
```

---

## 🎮 Have Fun!

Your TurtleBot3 is ready to explore the maze! Start with the simple navigation to see it in action, then try the full autonomous system for advanced features.

**Pro tip:** Watch the terminal output to see the robot's decision-making process in real-time!
