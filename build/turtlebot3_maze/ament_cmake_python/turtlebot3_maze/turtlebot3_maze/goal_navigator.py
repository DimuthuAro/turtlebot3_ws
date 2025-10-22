#!/usr/bin/env python3
"""
TurtleBot3 Goal Navigator using LiDAR
Autonomous navigation to a goal position using SLAM and Nav2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import math


class GoalNavigator(Node):
    """Navigate TurtleBot3 to a goal position using LiDAR and Nav2"""
    
    def __init__(self):
        super().__init__('goal_navigator')
        
        # Goal position (matching the green sphere in the world)
        self.goal_x = 5.0
        self.goal_y = 5.0
        
        # Create action client for Nav2
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # Subscribe to LiDAR data for monitoring
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # State tracking
        self.navigating = False
        self.lidar_data = None
        
        self.get_logger().info('Goal Navigator initialized')
        self.get_logger().info(f'Target goal: ({self.goal_x}, {self.goal_y})')
        
        # Don't wait for server in __init__, do it in send_goal instead
        self.server_ready = False
        
    def scan_callback(self, msg):
        """Process LiDAR scan data"""
        self.lidar_data = msg
        
        # Find minimum distance to obstacles
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        if valid_ranges:
            min_distance = min(valid_ranges)
            if min_distance < 0.3:  # 30cm safety threshold
                self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m!')
    
    def send_goal(self):
        """Send navigation goal to Nav2"""
        if self.navigating:
            self.get_logger().warn('Already navigating to a goal')
            return
        
        # Wait for action server with timeout
        if not self.server_ready:
            self.get_logger().info('Waiting for Nav2 action server...')
            timeout_sec = 30.0
            server_available = self._action_client.wait_for_server(timeout_sec=timeout_sec)
            
            if not server_available:
                self.get_logger().error(f'Nav2 action server not available after {timeout_sec} seconds!')
                return
            
            self.server_ready = True
            self.get_logger().info('Nav2 action server ready!')
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        
        # Set goal pose
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.position.z = 0.0
        
        # Orientation (facing forward)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'Sending goal to Nav2: ({self.goal_x}, {self.goal_y})')
        
        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.navigating = True
    
    def goal_response_callback(self, future):
        """Handle goal acceptance response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by Nav2!')
            self.navigating = False
            return
        
        self.get_logger().info('Goal accepted by Nav2! Robot is navigating...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Process navigation feedback"""
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        
        # Calculate distance to goal
        dx = self.goal_x - current_pose.position.x
        dy = self.goal_y - current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        
        self.get_logger().info(
            f'Distance to goal: {distance:.2f}m | '
            f'Position: ({current_pose.position.x:.2f}, {current_pose.position.y:.2f})'
        )
    
    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.navigating = False
        
        if result:
            self.get_logger().info('🎉 SUCCESS! Robot reached the goal! 🎉')
        else:
            self.get_logger().error('❌ Navigation failed!')


def main(args=None):
    rclpy.init(args=args)
    
    navigator = GoalNavigator()
    
    # Wait a bit for everything to initialize
    import time
    time.sleep(2.0)
    
    # Send the navigation goal
    navigator.send_goal()
    
    # Spin to process callbacks
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Navigation interrupted by user')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
