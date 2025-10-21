# TurtleBot3 Autonomous Goal Navigation

## 🎯 Overview

This system enables a TurtleBot3 robot to autonomously navigate through a **square maze** to find a **goal destination** using **LiDAR sensor** data.

## 🗺️ Maze Description

- **Type**: Square maze (6x6 meters)
- **Starting Position**: (0.5, 0.5) - Bottom-left corner
- **Goal Position**: (5.0, 5.0) - Top-right corner (marked with green sphere)
- **Boundaries**: Blue walls
- **Internal Obstacles**: Red walls creating maze structure
- **Passages**: Wide enough for TurtleBot3 navigation

## 🤖 Robot Capabilities

The TurtleBot3 Burger robot uses:
- **360° LiDAR Sensor**: Scans environment up to 3.5 meters
- **SLAM (Simultaneous Localization and Mapping)**: Builds real-time map
- **Nav2 Navigation Stack**: Plans optimal path and executes movement
- **Obstacle Avoidance**: Dynamically avoids walls detected by LiDAR

## 🚀 Quick Start

### Run Autonomous Navigation

```bash
cd ~/turtlebot3_ws
./run_autonomous_goal.sh
```

This single command will:
1. Launch Gazebo with the square maze world
2. Spawn TurtleBot3 at starting position (0.5, 0.5)
3. Start SLAM Toolbox for real-time mapping
4. Launch Nav2 for path planning
5. Open RViz for visualization
6. **Automatically** send the robot to the goal after 10 seconds

### What You'll See

**Gazebo Window**:
- Square maze with blue boundary walls
- Red internal maze walls
- TurtleBot3 robot at start position
- **Green sphere** marking the goal at (5, 5)

**RViz Window**:
- Real-time map being built by SLAM
- Robot's current position
- Planned path (green line)
- Global costmap (obstacles and safety zones)
- Local costmap (immediate surroundings)
- LiDAR scan visualization (red dots)

### Expected Behavior

1. **0-10 seconds**: System initialization
   - Robot spawns at (0.5, 0.5)
   - SLAM starts building map
   - Nav2 initializes planning system

2. **10+ seconds**: Autonomous navigation begins
   - Goal Navigator sends target (5.0, 5.0) to Nav2
   - Nav2 plans optimal path through maze
   - Robot starts moving toward goal
   - LiDAR continuously scans for obstacles
   - Map updates in real-time

3. **During navigation**:
   - Robot follows planned path
   - Avoids walls detected by LiDAR
   - Replans if new obstacles detected
   - Distance to goal shown in terminal

4. **Goal reached**:
   - Terminal displays: "🎉 SUCCESS! Robot reached the goal! 🎉"
   - Robot stops at green sphere location

## 📊 Monitoring Progress

### Terminal Output

```
[goal_navigator]: Distance to goal: 6.36m | Position: (0.50, 0.50)
[goal_navigator]: Distance to goal: 5.12m | Position: (1.23, 0.89)
[goal_navigator]: Distance to goal: 3.45m | Position: (2.56, 2.34)
...
[goal_navigator]: 🎉 SUCCESS! Robot reached the goal! 🎉
```

### RViz Visualization

- **Green line**: Planned path from current position to goal
- **Red dots**: LiDAR scan points (walls/obstacles)
- **Blue/Purple regions**: Costmaps (robot won't drive here)
- **Robot icon**: Current position and orientation
- **Map**: White = free space, Black = obstacles, Gray = unknown

## 🛠️ System Architecture

### Components

1. **Gazebo**: Physics simulation and 3D visualization
2. **TurtleBot3**: Simulated robot with LiDAR sensor
3. **SLAM Toolbox**: Creates map from LiDAR data
4. **Nav2**: Navigation stack (path planning, control)
5. **Goal Navigator**: Python node that sends goal coordinates
6. **RViz**: Visualization of map, robot, and navigation

### Data Flow

```
LiDAR Sensor → SLAM Toolbox → Map
                     ↓
Goal Navigator → Nav2 → Path Planner → Controller → Robot Motors
                     ↓
                 RViz (Visualization)
```

## 📁 Key Files

### Launch Files
- `autonomous_navigation.launch.py`: Complete autonomous system
- `slam.launch.py`: SLAM mapping only (for manual testing)
- `navigation.launch.py`: Nav2 navigation only

### Python Scripts
- `goal_navigator.py`: Autonomous goal-seeking behavior
- `dqn_navigation.py`: Deep Q-Learning navigation (alternative)

### Configuration Files
- `config/nav2_params.yaml`: Nav2 navigation parameters
- `config/slam_params.yaml`: SLAM Toolbox settings

### World Files
- `worlds/square_maze.world`: 6x6 square maze with goal marker
- `worlds/maze_assignment.world`: Complex random maze
- `worlds/maze_simple.world`: Simple grid maze

## 🎮 Manual Control (Optional)

If you want to manually control the robot before automatic navigation:

```bash
# Terminal 1: Launch SLAM only
ros2 launch turtlebot3_maze slam.launch.py

# Terminal 2: Teleop control
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

Then later send a goal manually in RViz:
- Click "2D Goal Pose" button
- Click on map where you want robot to go
- Robot will navigate there autonomously

## ⚙️ Customization

### Change Goal Position

Edit `goal_navigator.py`:
```python
self.goal_x = 5.0  # Change X coordinate
self.goal_y = 5.0  # Change Y coordinate
```

### Change Starting Position

Edit `autonomous_navigation.launch.py`:
```python
'x_pose': '0.5',  # Change X coordinate
'y_pose': '0.5',  # Change Y coordinate
```

### Use Different Maze

Edit `autonomous_navigation.launch.py`:
```python
world_file = PathJoinSubstitution([
    turtlebot3_maze_dir,
    'worlds',
    'maze_simple.world'  # Change world file
])
```

## 🐛 Troubleshooting

### Robot doesn't move
- Wait 10 seconds for Nav2 to initialize
- Check terminal for error messages
- Ensure goal position is reachable

### Robot gets stuck
- Nav2 will replan the path
- If stuck for >30 seconds, press Ctrl+C and restart
- Check if maze passages are wide enough

### No map appearing in RViz
- SLAM needs time to build initial map
- Ensure LiDAR is detecting walls
- Check that robot is not in completely open space

### Navigation fails
- Goal might be unreachable (blocked by walls)
- Map might be incomplete
- Try increasing navigation timeout in Nav2 params

## 📚 Additional Resources

- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)

## 🎓 Assignment Notes

This system demonstrates:
- ✅ LiDAR-based obstacle detection
- ✅ Autonomous navigation to goal
- ✅ Real-time mapping (SLAM)
- ✅ Path planning around obstacles
- ✅ Square maze environment
- ✅ Visible goal marker

Perfect for IE4060 Assignment 02!
