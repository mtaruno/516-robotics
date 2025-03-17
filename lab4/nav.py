#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import time
import sys
from rclpy.duration import Duration


class NavigationPoseTimer(Node):
    def __init__(self):
        super().__init__('navigation_pose_timer')
        
        # Create an action client for the NavigateToPose action
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # List of poses (x, y, z, qx, qy, qz, qw)
        self.poses = [
            (-2.0, -0.5, 0.0, 0.0, 0.0, 0.0, 1.0),     # p0
            (0.0, -2.0, 0.0, 0.0, 0.0, 0.342, 0.939),  # p1
            (2.0, 0.0, 0.0, 0.0, 0.0, 0.472, 0.882),   # p2
            (0.55, 1.25, 0.0, 0.0, 0.0, -0.707, 0.707) # p3
        ]
        
        self.current_pose_index = 0
        self.total_poses = len(self.poses)
        self.start_time = None
        
        # Wait for the action server to become available
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available after waiting')
            sys.exit(1)
        self.get_logger().info('Action server available!')
        
        # Start the navigation sequence
        self.send_next_goal()
    
    def send_next_goal(self):
        if self.current_pose_index < self.total_poses:
            pose = self.poses[self.current_pose_index]
            self.get_logger().info(f'Navigating to pose p{self.current_pose_index}: {pose}')
            
            # Create and populate a goal message
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            
            # Set the position
            goal_msg.pose.pose.position.x = pose[0]
            goal_msg.pose.pose.position.y = pose[1]
            goal_msg.pose.pose.position.z = pose[2]
            
            # Set the orientation (quaternion)
            goal_msg.pose.pose.orientation.x = pose[3]
            goal_msg.pose.pose.orientation.y = pose[4]
            goal_msg.pose.pose.orientation.z = pose[5]
            goal_msg.pose.pose.orientation.w = pose[6]
            
            # Record the start time for this pose
            self.start_time = time.time()
            
            # Send the goal
            self.nav_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().info('Navigation sequence completed!')
            rclpy.shutdown()
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        
        # Get the result
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        end_time = time.time()
        elapsed_time = end_time - self.start_time
        
        self.get_logger().info(f'Reached pose p{self.current_pose_index} in {elapsed_time:.2f} seconds')
        
        # Move to the next pose
        self.current_pose_index += 1
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationPoseTimer()
    rclpy.spin(node)
    

if __name__ == '__main__':
    main()