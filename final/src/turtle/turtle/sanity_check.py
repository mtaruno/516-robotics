#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBotSanityCheck(Node):
    def __init__(self):
        super().__init__('sanity_check')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(1)  # Allow time for the publisher to set up
        self.run_test()

    def run_test(self):
        self.get_logger().info("Publishing forward velocity for 5 seconds...")

        twist = Twist()
        twist.linear.x = 0.2  # Move forward at 0.2 m/s
        twist.angular.z = 0.0  # No rotation

        start_time = time.time()
        while time.time() - start_time < 5:  # Move for 5 seconds
            self.publisher_.publish(twist)
            time.sleep(0.1)  # 10 Hz update rate

        # Stop the robot
        twist.linear.x = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info("Stopping movement. Test complete.")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotSanityCheck()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
