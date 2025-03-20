#!/usr/bin/env python3


import rclpy
import time
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose, Twist


# 10.18.3.90

class TurtleBot3Controller(Node):
    """Handles linear and angular movement with speed control and acceleration."""
    
    MAX_SPEED = 0.24  # Set max allowed speed (TurtleBot3 firmware limit)
    ACCELERATION_STEP = 0.02  # Gradual increase per step

    def __init__(self):
        super().__init__('turtlebot3_controller')

        # Declare parameters
        self.declare_parameter('linear_vel', 0.2)  # Default 0.2 m/s
        self.declare_parameter('angular_vel', 0.0)  # Default 0 rad/s
        self.declare_parameter('duration', 5.0)  # Default 5 seconds

        # Create publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Ensure node has initialized before sending commands
        time.sleep(1)

    def move(self):
        """Executes movement with acceleration control."""
        
        # Fetch parameters
        target_linear_velocity = self.get_parameter('linear_vel').value
        angular_velocity = self.get_parameter('angular_vel').value
        duration = self.get_parameter('duration').value

        # Enforce speed limit
        if target_linear_velocity > self.MAX_SPEED:
            self.get_logger().warn(f"⚠️ Speed {target_linear_velocity} m/s exceeds max limit! Setting to {self.MAX_SPEED} m/s.")
            target_linear_velocity = self.MAX_SPEED

        # Validate duration
        if duration <= 0:
            self.get_logger().warn("⚠️ Duration must be greater than 0! Setting to 1 second.")
            duration = 1.0

        # Log movement settings
        self.get_logger().info(f"🚀 Moving: Linear={target_linear_velocity} m/s, Angular={angular_velocity} rad/s, Duration={duration}s")

        msg = Twist()
        current_speed = 0.0  # Start from 0 speed
        start_time = time.time()

        # **Accelerate Gradually** to Target Speed
        while time.time() - start_time < duration:
            # Increase or decrease speed gradually
            if current_speed < target_linear_velocity:
                current_speed += self.ACCELERATION_STEP
                if current_speed > target_linear_velocity:
                    current_speed = target_linear_velocity
            elif current_speed > target_linear_velocity:
                current_speed -= self.ACCELERATION_STEP
                if current_speed < target_linear_velocity:
                    current_speed = target_linear_velocity

            msg.linear.x = current_speed
            msg.angular.z = angular_velocity
            self.publisher_.publish(msg)
            self.get_logger().info(f"🔄 Accelerating: Speed={current_speed:.2f} m/s")

            time.sleep(0.1)  # Ensure consistent updates

        # **Stop the Robot After Duration Ends**
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info("🛑 Stopping robot.")


def execute_movement_sequence(movement_list):
    """Executes a sequence of TurtleBot3 movements."""
    for move in movement_list:
        controller = TurtleBot3Controller()
        controller.set_parameters([
            Parameter("linear_vel", Parameter.Type.DOUBLE, move["linear_vel"]),
            Parameter("angular_vel", Parameter.Type.DOUBLE, move["angular_vel"]),
            Parameter("duration", Parameter.Type.DOUBLE, move["duration"]),
        ])
        controller.move()
        controller.destroy_node()
        time.sleep(1)
    time.sleep(3)



def main():
    rclpy.init()

    movements_before = [
    {"linear_vel": -0.2, "angular_vel": 0.0, "duration": 16.0},
    # {"linear_vel": -0.2, "angular_vel": 0.0, "duration": 16.0}
    ]
    execute_movement_sequence(movements_before)


if __name__ == "__main__":
    main()





# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import time

# class TurtleBotController(Node):
#     def __init__(self):
#         super().__init__('turtlebot3_turn_script')
#         self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.rate = 0.1 # self.create_rate(10)  # 10 Hz

#     def move_forward(self, duration, speed):
#         """ Moves the robot forward for a specified duration. """
#         self.get_logger().info(f"Moving forward for {duration:.2f} seconds at {speed:.2f} m/s")
#         twist = Twist()
#         twist.linear.x = speed
#         twist.angular.z = 0.0
#         start_time = time.time()

#         while time.time() - start_time < duration:
#             self.publisher_.publish(twist)
#             self.get_logger().info("Publishing forward movement...")
#             time.sleep(self.rate)

#         # Stop the robot
#         twist.linear.x = 0.0
#         self.publisher_.publish(twist)
#         self.get_logger().info("Stopping forward movement.")

#     def rotate(self, angle, duration):
#         """ Rotates the robot by a given angle over a specified duration. """
#         self.get_logger().info(f"Rotating by {angle:.2f} radians over {duration:.2f} seconds")
#         twist = Twist()
#         twist.linear.x = 0.0
#         twist.angular.z = angle / duration if duration > 0 else 0
#         start_time = time.time()

#         while time.time() - start_time < duration:
#             self.publisher_.publish(twist)
#             self.get_logger().info("Publishing rotation movement...")
#             time.sleep(self.rate)

#         # Stop rotation
#         twist.angular.z = 0.0
#         self.publisher_.publish(twist)
#         self.get_logger().info("Stopping rotation.")

# def main(args=None):
#     rclpy.init(args=args)
#     node = TurtleBotController()

#     move_turn_sequences = [
#         {"forward_duration": 3.0, "forward_speed": 0.2, "turn_angle": 1.57},  # 90-degree right
#         {"forward_duration": 2.5, "forward_speed": 0.15, "turn_angle": -1.57},  # 90-degree left
#         {"forward_duration": 4.0, "forward_speed": 0.2, "turn_angle": 1.57}   # 90-degree right
#     ]

#     angular_speed = 0.5  # rad/s

#     node.move_forward(16,0.2)

#     time.sleep(2)

#     node.rotate(angle = 4.1,duration=4)
#     # for i, sequence in enumerate(move_turn_sequences):
#     #     node.get_logger().info(f"Executing Sequence {i+1}")
#     #     node.move_forward(sequence["forward_duration"], sequence["forward_speed"])
#     #     time.sleep(0.5)

#     #     turn_duration = abs(sequence["turn_angle"]) / angular_speed
#     #     node.rotate(sequence["turn_angle"], turn_duration)
#     #     time.sleep(0.5)

#     node.get_logger().info("Movement sequence complete.")

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     try:
#         main()
#     except KeyboardInterrupt:
#         print('Interrupted')
