#!/bin/bash
# Simple test script to verify SLAM system

echo "================================================"
echo "TurtleBot3 SLAM Test - Simplified Launch"
echo "================================================"

# Source workspace
source /home/ayesh/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

echo "Step 1: Launching Gazebo with maze..."
echo "================================================"

# Launch Gazebo with the maze world
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py x_pose:=1.0 y_pose:=1.0 world:=/home/ayesh/turtlebot3_ws/install/turtlebot3_maze/share/turtlebot3_maze/worlds/maze_assignment.world &
GAZEBO_PID=$!

sleep 5

echo ""
echo "Step 2: Waiting for Gazebo to be ready..."
echo "================================================"
sleep 10

echo ""
echo "Step 3: Launching SLAM Toolbox..."
echo "================================================"

# Launch SLAM toolbox
ros2 launch slam_toolbox online_async_launch.py params_file:=/home/ayesh/turtlebot3_ws/src/turtlebot3_maze/config/slam_params.yaml use_sim_time:=true &
SLAM_PID=$!

sleep 3

echo ""
echo "Step 4: Launching RViz..."
echo "================================================"

# Launch RViz
ros2 run rviz2 rviz2 -d /home/ayesh/turtlebot3_ws/src/turtlebot3_maze/rviz/slam.rviz &
RVIZ_PID=$!

echo ""
echo "================================================"
echo "System launched successfully!"
echo "================================================"
echo "- Gazebo PID: $GAZEBO_PID"
echo "- SLAM PID: $SLAM_PID"
echo "- RViz PID: $RVIZ_PID"
echo ""
echo "Press Ctrl+C to stop all processes..."

# Wait for user interrupt
trap "kill $GAZEBO_PID $SLAM_PID $RVIZ_PID 2>/dev/null" EXIT
wait
