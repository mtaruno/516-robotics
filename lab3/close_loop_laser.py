#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LaserCloseLoop(Node):
    def __init__(self):
        super().__init__('laser_closeloop')
        self.debug = True
        # Subscribe to the /scan topic.
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # Publisher for velocity commands on /cmd_vel.
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Use a variable to store the initial front distance.
        self.initial_front = None
        self.motion_move = Twist()
        self.motion_stop = Twist()
        # Set a constant forward speed.
        self.motion_move.linear.x = 0.15  # m/s
        # Set stop command.
        self.motion_stop.linear.x = 0.0

    def scan_callback(self, msg):
        # Assume the "front" of the robot is the center index of the scan array.
        front_index = len(msg.ranges) // 2
        current_front = msg.ranges[front_index]
        if self.initial_front is None:
            self.initial_front = current_front
            self.get_logger().info(f"Initial front distance: {self.initial_front:.2f} m")
            self.pub.publish(self.motion_move)
            return

        distance_traveled = self.initial_front - current_front
        if distance_traveled >= 1.5:
            self.pub.publish(self.motion_stop)
            self.get_logger().info("Reached 1.5 m (laser): stopping...")
            rclpy.shutdown()
        else:
            self.pub.publish(self.motion_move)
            if self.debug:
                self.get_logger().info(f"Current front: {current_front:.2f} m, Distance traveled: {distance_traveled:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = LaserCloseLoop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

