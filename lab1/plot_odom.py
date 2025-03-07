#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # Import the Odometry message type
import matplotlib.pyplot as plt

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        # Create a subscription to the '/odom' topic with a queue size of 10
        self.subscription = self.create_subscription(
            Odometry,             # Message type
            '/odom',              # Topic name
            self.odom_callback,   # Callback function to process messages
            10                    # Queue size
        )
        self.subscription  # Prevent unused variable warning

        # Lists to store the x and y data for plotting
        self.x_data = []
        self.y_data = []

    def odom_callback(self, msg):
        # Retrieve the position from the odometry message:
        # The pose information is contained in msg.pose.pose.position.
        position = msg.pose.pose.position
        x = position.x
        y = position.y

        # Log the received position
        self.get_logger().info(f"Position -> x: {x}, y: {y}")

        # Append the x and y positions to the respective lists
        self.x_data.append(x)
        self.y_data.append(y)

    def plot_data(self):
        # Plot the odometry data using matplotlib
        plt.figure()
        plt.plot(self.x_data, self.y_data, label='Odometry Path')
        plt.title('Odometry X-Y Path')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.legend()
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()

    try:
        # Spin the node so callbacks are processed until a KeyboardInterrupt occurs.
        rclpy.spin(odom_subscriber)
    except KeyboardInterrupt:
        # When the node is interrupted, plot the data
        odom_subscriber.plot_data()

    odom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

