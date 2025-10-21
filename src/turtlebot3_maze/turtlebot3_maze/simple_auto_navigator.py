#!/usr/bin/env python3
"""
Simple Autonomous Navigator for TurtleBot3
Uses LiDAR to navigate toward goal while avoiding obstacles
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np


class SimpleAutoNavigator(Node):
    """Navigate TurtleBot3 to goal using LiDAR obstacle avoidance"""
    
    def __init__(self):
        super().__init__('simple_auto_navigator')
        
        # Goal position
        self.goal_x = 5.0
        self.goal_y = 5.0
        
        # Current robot position
        self.current_x = 0.5
        self.current_y = 0.5
        self.current_yaw = 0.0
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Control parameters
        self.linear_speed = 0.15
        self.angular_speed = 0.5
        self.obstacle_distance = 0.4  # Stop if obstacle within 40cm
        self.goal_tolerance = 0.3  # Consider goal reached within 30cm
        
        # LiDAR data
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        
        # State
        self.goal_reached = False
        
        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('🤖 Simple Auto Navigator Started!')
        self.get_logger().info(f'📍 Current: ({self.current_x:.2f}, {self.current_y:.2f})')
        self.get_logger().info(f'🎯 Goal: ({self.goal_x:.2f}, {self.goal_y:.2f})')
        
    def odom_callback(self, msg):
        """Update current position from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def scan_callback(self, msg):
        """Process LiDAR scan data"""
        ranges = np.array(msg.ranges)
        ranges[ranges == 0] = float('inf')  # Ignore zero readings
        
        # Divide scan into regions
        num_readings = len(ranges)
        
        # Front: center ±30 degrees
        front_start = num_readings - 30
        front_end = 30
        front_ranges = np.concatenate([ranges[front_start:], ranges[:front_end]])
        self.front_distance = np.min(front_ranges[np.isfinite(front_ranges)]) if np.any(np.isfinite(front_ranges)) else float('inf')
        
        # Left: 60-120 degrees
        left_start = 60
        left_end = 120
        left_ranges = ranges[left_start:left_end]
        self.left_distance = np.min(left_ranges[np.isfinite(left_ranges)]) if np.any(np.isfinite(left_ranges)) else float('inf')
        
        # Right: 240-300 degrees
        right_start = 240
        right_end = 300
        right_ranges = ranges[right_start:right_end]
        self.right_distance = np.min(right_ranges[np.isfinite(right_ranges)]) if np.any(np.isfinite(right_ranges)) else float('inf')
    
    def control_loop(self):
        """Main control loop"""
        if self.goal_reached:
            return
        
        # Calculate distance and angle to goal
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance_to_goal = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - self.current_yaw)
        
        # Check if goal reached
        if distance_to_goal < self.goal_tolerance:
            self.stop_robot()
            self.goal_reached = True
            self.get_logger().info('🎉🎉🎉 GOAL REACHED! 🎉🎉🎉')
            self.get_logger().info(f'Final position: ({self.current_x:.2f}, {self.current_y:.2f})')
            return
        
        # Log progress every 1 second
        if hasattr(self, '_last_log_time'):
            if (self.get_clock().now().nanoseconds - self._last_log_time) > 1e9:
                self.get_logger().info(
                    f'📍 Distance to goal: {distance_to_goal:.2f}m | '
                    f'Front: {self.front_distance:.2f}m | '
                    f'Left: {self.left_distance:.2f}m | '
                    f'Right: {self.right_distance:.2f}m'
                )
                self._last_log_time = self.get_clock().now().nanoseconds
        else:
            self._last_log_time = self.get_clock().now().nanoseconds
        
        # Create velocity command
        cmd = Twist()
        
        # Obstacle avoidance logic
        if self.front_distance < self.obstacle_distance:
            # Obstacle in front - turn away
            self.get_logger().warn('⚠️  Obstacle ahead! Turning...')
            cmd.linear.x = 0.0
            # Turn toward the side with more space
            if self.left_distance > self.right_distance:
                cmd.angular.z = self.angular_speed
            else:
                cmd.angular.z = -self.angular_speed
        
        elif abs(angle_diff) > 0.2:
            # Need to rotate toward goal
            cmd.linear.x = 0.05  # Move slowly while rotating
            cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        
        else:
            # Move toward goal
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.3 * angle_diff  # Slight correction
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
    
    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    
    navigator = SimpleAutoNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Navigation interrupted by user')
    finally:
        navigator.stop_robot()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
