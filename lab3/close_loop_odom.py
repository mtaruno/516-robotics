#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class OdomCloseLoop(Node):
    def __init__(self):
        super().__init__('close_loop_odom')
        self.debug = True
        # Subscribe to the /odom topic.
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # Publisher for velocity commands on /cmd_vel.
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Use a variable to store the initial x position.
        self.initial_x = None
        self.motion_move = Twist()
        self.motion_stop = Twist()
        # Set a constant forward speed.
        self.motion_move.linear.x = 0.15  # m/s
        # Set stop command.
        self.motion_stop.linear.x = 0.0

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        if self.initial_x is None:
            self.initial_x = current_x
            self.get_logger().info(f"Initial x position: {self.initial_x:.2f} m")
            self.pub.publish(self.motion_move)
            return

        distance_traveled = current_x - self.initial_x
        if distance_traveled >= 1.5:
            self.pub.publish(self.motion_stop)
            self.get_logger().info("Reached 1.5 m (odom): stopping...")
            rclpy.shutdown()
        else:
            self.pub.publish(self.motion_move)
            if self.debug:
                self.get_logger().info(f"Current x: {current_x:.2f} m, Distance traveled: {distance_traveled:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = OdomCloseLoop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

