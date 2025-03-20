import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBotVelocityController(Node):
    def __init__(self):
        super().__init__('turtlebot_velocity_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)  # 10 Hz
        self.start_time = time.time()
        self.duration = 5.0  # Move for 5 seconds
        self.moving = True

    def publish_velocity(self):
        if time.time() - self.start_time > self.duration:
            self.moving = False

        vel_msg = Twist()
        if self.moving:
            vel_msg.linear.x = 0.2  # Move forward
            vel_msg.angular.z = 0.5  # Rotate
        else:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0

        self.publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotVelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("Turtle activated")
    main()
