#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from pyquaternion import Quaternion as PyQuaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper  
import numpy as np

 
def execute_robotic_transfer(robot_arm, gripper_control, approach_position, grasp_position, 
                           transit_position1, transit_position2, release_position1, 
                           release_position2, release_position3):
    robot_arm.inverse_kinematic_movement(approach_position)
    time.sleep(0.5)
    # gripper_control.move_to_position(0.0)
    gripper_control.open()
    robot_arm.inverse_kinematic_movement(grasp_position)
    # gripper_control.move_to_position(0.8)
    gripper_control.close()
    print("got the cube")
    time.sleep(0.5)
    robot_arm.inverse_kinematic_movement(transit_position1)
    time.sleep(0.5)
    robot_arm.inverse_kinematic_movement(transit_position2)
    time.sleep(0.5)
    robot_arm.inverse_kinematic_movement(release_position1)
    time.sleep(0.5)
    robot_arm.inverse_kinematic_movement(release_position2)
    time.sleep(0.5)
    robot_arm.inverse_kinematic_movement(release_position3)
    time.sleep(0.5)
    # gripper_control.move_to_position(0.0)
    gripper_control.open()
    print("finished!")

def create_robot_pose(position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w):
    new_pose = Pose()
    new_pose.position.x = position_x
    new_pose.position.y = position_y
    new_pose.position.z = position_z
    new_pose.orientation.x = orientation_x
    new_pose.orientation.y = orientation_y
    new_pose.orientation.z = orientation_z
    new_pose.orientation.w = orientation_w
    return new_pose

class TurtleBotController(Node):
    def __init__(self, speed=0.1, distance=1.0):
        super().__init__('turtlebot_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = speed  # Set the speed (positive for moving forward)
        self.target_distance = distance  # Set the target distance to move
        self.movement_complete = False

 

    # Function to move the TurtleBot and display progress in meters
    def move(self):
        print("Starting to move forward...")

 

        twist = Twist()
        twist.linear.x = self.speed  # Set the forward speed

 

        # Calculate total duration and setup for progress display
        duration = self.target_distance / abs(self.speed)
        start_time = time.time()

 

        # Initialize distance tracking
        traveled_distance = 0.0

 

        # Publish speed command and display progress
        while not self.movement_complete and (time.time() - start_time) < duration:
            self.cmd_vel_publisher.publish(twist)
            elapsed_time = time.time() - start_time
            traveled_distance = abs(self.speed) * elapsed_time

 

            # Print progress in meters
            print(f"Moved: {traveled_distance:.2f} meters", end='\r')

            # Add a small delay to avoid overloading the command buffer
            time.sleep(0.1)

 

        # Stop the TurtleBot
        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.movement_complete = True

        print(f"\nFinished! Moved a total of {traveled_distance:.2f} meters.")



def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create first TurtleBot controller and execute movement
    node = TurtleBotController(speed=0.2, distance=3.375)
    node.move()
    
    # Setup gripper and arm controllers
    gripper = Gen3LiteGripper()
    arm = Gen3LiteArm()
    arm.go_vertical()
    
    # Define waypoint positions
    approach_position = create_robot_pose(0.370, -0.092, 0.371, 0.817, 0.574, -0.036, 0.028)
    # Commented out alternative position
    # alt_approach_position = create_robot_pose(0.380, -0.030, 0.371, 0.817, 0.574, -0.036, 0.028)
    
    # Modified grasp position coordinates
    grasp_position = create_robot_pose(0.370, -0.070, 0.130, 0.758, 0.652, 0.008, 0.017)
    # Commented out alternative position
    # alt_grasp_position = create_robot_pose(0.394, -0.031, 0.161, 0.751, 0.659, -0.025, 0.017)
    
    transit_position1 = create_robot_pose(-.293, 0.081, 0.228, 0.752, 0.658, 0.023, -0.027)
    transit_position2 = create_robot_pose(-0.174, -0.002, 0.191, 0.494, 0.867, -0.007, -0.066)
    
    release_position1 = create_robot_pose(-0.195, 0.034, -0.164, 0.494, 0.867, -0.008, -0.066)
    release_position2 = create_robot_pose(-0.173, 0.0, -0.342, 0.542, 0.838, -0.04, -0.042)
    release_position3 = create_robot_pose(-0.159, 0.085, -0.336, -0.018, 0.997, 0.026, -0.076)
    
    # Note: standard movement takes 2min 5s
    
    # Execute the pick and place operation
    execute_robotic_transfer(arm, gripper, approach_position, grasp_position, 
                           transit_position1, transit_position2, release_position1, 
                           release_position2, release_position3)

    # Create second TurtleBot controller with reverse movement
    node2 = TurtleBotController(speed=-0.1, distance=3.2)
    
    # Execute the reverse move command
    node2.move()

    # Clean up nodes
    node.destroy_node()
    node2.destroy_node()

    # Shutdown ROS2 and clean up all resources
    rclpy.shutdown()
    gripper.shutdown()
    arm.shutdown()


if __name__ == '__main__':
    main()
# """
# pick_and_place_over_obstacle.py

# This script demonstrates a pick‐and‐place routine for a Kinova Gen3 Lite robot arm
# with a gripper. The routine:
#   1. Moves to the grasp (pick) pose where the cube is located.
#   2. Closes the gripper to grasp the cube.
#   3. Moves to an intermediate pose (after pick-up).
#   4. Moves to a transfer pose to clear an obstacle.
#   5. Moves to the place (release) pose and opens the gripper to release the cube.
#   6. (Optionally) Retreats after releasing the cube.
# Adjust the pose positions and orientations as needed for your environment.
# """

# import time
# import rclpy
# from geometry_msgs.msg import Pose
# # Import the interfaces – ensure that the module paths are correct for your setup.
# from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper


# def pick_and_place_over_obstacle(arm, gripper):
#     # Open the gripper to prepare for grasping.
#     arm.node.get_logger().info("Opening gripper...")
#     gripper.open()

#     time.sleep(1)
# # 10.18.2.230
#     # ------------------------------
#     # Define the Pick (Grasp) Pose
#     # ------------------------------

#     # New poses with the rightmost table
#     poses = [
#     [[0.461, 0.037, 0.107], [0.958, 0.013, -0.288, 0.007]],
#     [[-0.392, -0.244, 0.105], [-0.238, 0.927, 0.077, 0.281]],
#     [[-0.298, -0.093, -0.198], [-0.129, 0.932, 0.035, 0.338]],
#     [[-0.210, -0.147, -0.255], [-0.340, 0.939, -0.040, -0.016]]
#     ]

#     # Initial poses
#     # poses = [
#     # [[0.463, -0.029, 0.110], [0.958, 0.013, -0.285, 0.007]],
#     # [[-0.397, -0.238, 0.108], [-0.297, 0.911, 0.093, 0.272]],
#     # [[-0.201, -0.138, -0.245], [-0.326, 0.862, 0.104, 0.374]],
#     # [[-0.210, -0.147, -0.255], [-0.340, 0.939, -0.040, -0.016]]
#     # ]

#     def make_pose(num):
#         p = Pose()
#         p.position.x = poses[num][0][0]
#         p.position.y = poses[num][0][1]
#         p.position.z = poses[num][0][2]
#         p.orientation.x = poses[num][1][0]
#         p.orientation.y = poses[num][1][1]
#         p.orientation.z = poses[num][1][2]
#         p.orientation.w = poses[num][1][3]
#         return p

#     p1 = make_pose(0)
#     p2 = make_pose(1)
#     p3 = make_pose(2)
#     p4 = make_pose(3)


#     def go_back():
#         arm.node.get_logger().info("coming back home baby")
#         # ------------------------------
#         # Execute the Pick and Place Sequence
#         # ------------------------------
#         # 1. Move directly to the pick (grasp) pose.
#         arm.node.get_logger().info("p4")
#         arm.inverse_kinematic_movement(p4)
#         time.sleep(1)
#         arm.node.get_logger().info("p3")
#         arm.inverse_kinematic_movement(p3)
#         time.sleep(1)

#         # 4. Move to the transfer pose to clear the obstacle.
#         arm.node.get_logger().info("p2")
#         arm.inverse_kinematic_movement(p2)
#         time.sleep(1)

#         # # 5. Move to the place (release) pose.
#         arm.node.get_logger().info("p1")
#         arm.inverse_kinematic_movement(p1)
#         time.sleep(1)

#         # # 6. Open the gripper to release the cube.
#         arm.node.get_logger().info("open again")
#         gripper.open()
#         time.sleep(1)

#     def go_forward():
#         # ------------------------------
#         # Execute the Pick and Place Sequence
#         # ------------------------------
#         # 1. Move directly to the pick (grasp) pose.
#         arm.node.get_logger().info("Moving to pick pose...")
#         arm.inverse_kinematic_movement(p1)
#         time.sleep(1)

#         # 2. Close the gripper to grasp the cube.
#         arm.node.get_logger().info("Grasping the cube (closing gripper)...")
#         gripper.close()
#         time.sleep(1)

#         # # 3. Move to the first intermediate pose after pick-up.
#         # arm.node.get_logger().info("Moving to p2...")
#         # arm.inverse_kinematic_movement(p2)
#         # time.sleep(1)

#         # # # 4. Move to the transfer pose to clear the obstacle.
#         # arm.node.get_logger().info("Moving to p3...")
#         # arm.inverse_kinematic_movement(p3)
#         # time.sleep(1)

#         # # # 5. Move to the place (release) pose.
#         # arm.node.get_logger().info("Moving to p4...")
#         # arm.inverse_kinematic_movement(p4)
#         # time.sleep(1)

#         # # 6. Open the gripper to release the cube.
#         arm.node.get_logger().info("Releasing the cube (opening gripper)...")
#         gripper.open()
#         time.sleep(1)
    


#     arm.go_vertical()
#     go_forward()

#     # go_back()

#     # # 7. (Optional) Retreat after releasing by staying at the place pose.
#     # arm.node.get_logger().info("Retreating from the place pose...")
#     # arm.inverse_kinematic_movement(place_pose)
#     # time.sleep(1)

#     arm.node.get_logger().info("Pick and place routine completed.")


# def main():
#     rclpy.init()
#     # Initialize the arm and gripper interfaces.
#     arm = Gen3LiteArm()
#     gripper = Gen3LiteGripper()
#     try:
#         pick_and_place_over_obstacle(arm, gripper)
#     except Exception as e:
#         arm.node.get_logger().error(f"Error during pick and place: {e}")
#     finally:
#         # Shutdown ROS2 and clean up.
#         rclpy.shutdown()
#         gripper.shutdown()
#         arm.shutdown()

# if __name__ == '__main__':
#     main()
