#!/usr/bin/env python3
"""
pick_and_place_over_obstacle.py

This script demonstrates a pick‐and‐place routine for a Kinova Gen3 Lite robot arm
with a gripper. The routine:
  1. Moves to the grasp (pick) pose where the cube is located.
  2. Closes the gripper to grasp the cube.
  3. Moves to an intermediate pose (after pick-up).
  4. Moves to a transfer pose to clear an obstacle.
  5. Moves to the place (release) pose and opens the gripper to release the cube.
  6. (Optionally) Retreats after releasing the cube.
Adjust the pose positions and orientations as needed for your environment.
"""

import time
import rclpy
from geometry_msgs.msg import Pose
# Import the interfaces – ensure that the module paths are correct for your setup.
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper


def pick_and_place_over_obstacle(arm, gripper):
    # Open the gripper to prepare for grasping.
    arm.node.get_logger().info("Opening gripper...")
    gripper.open()
    time.sleep(1)

    # ------------------------------
    # Define the Pick (Grasp) Pose
    # ------------------------------
    pick_pose = Pose()
    pick_pose.position.x = 0.463
    pick_pose.position.y = -0.029
    pick_pose.position.z = 0.110
    pick_pose.orientation.x = 0.958
    pick_pose.orientation.y = 0.013
    pick_pose.orientation.z = -0.285
    pick_pose.orientation.w = 0.007

    # ------------------------------
    # Define the First Intermediate Pose (after pick-up)
    # ------------------------------
    intermediate_pose = Pose()
    intermediate_pose.position.x = 0.181
    intermediate_pose.position.y = 0.352
    intermediate_pose.position.z = 0.444
    intermediate_pose.orientation.x = 0.684
    intermediate_pose.orientation.y = 0.525
    intermediate_pose.orientation.z = 0.442
    intermediate_pose.orientation.w = -0.248

    # ------------------------------
    # Define the Transfer Pose (over the obstacle)
    # ------------------------------
    transfer_pose = Pose()
    transfer_pose.position.x = 0.438
    transfer_pose.position.y = -0.198
    transfer_pose.position.z = 0.356
    transfer_pose.orientation.x = 0.911
    transfer_pose.orientation.y = -0.309
    transfer_pose.orientation.z = 0.019
    transfer_pose.orientation.w = -0.273

    # ------------------------------
    # Define the Place (Release) Pose
    # ------------------------------
    place_pose = Pose()
    place_pose.position.x = 0.247
    place_pose.position.y = 0.414
    place_pose.position.z = 0.120
    place_pose.orientation.x = 0.828
    place_pose.orientation.y = 0.514
    place_pose.orientation.z = -0.214
    place_pose.orientation.w = 0.060

    # ------------------------------
    # Execute the Pick and Place Sequence
    # ------------------------------

    # 1. Move directly to the pick (grasp) pose.
    arm.node.get_logger().info("Moving to pick pose...")
    arm.inverse_kinematic_movement(pick_pose)
    time.sleep(1)

    # 2. Close the gripper to grasp the cube.
    arm.node.get_logger().info("Grasping the cube (closing gripper)...")
    gripper.close()
    time.sleep(1)

    # 3. Move to the first intermediate pose after pick-up.
    arm.node.get_logger().info("Moving to first intermediate pose...")
    arm.inverse_kinematic_movement(intermediate_pose)
    time.sleep(1)

    # 4. Move to the transfer pose to clear the obstacle.
    arm.node.get_logger().info("Moving to transfer pose (over the obstacle)...")
    arm.inverse_kinematic_movement(transfer_pose)
    time.sleep(1)

    # 5. Move to the place (release) pose.
    arm.node.get_logger().info("Moving to place pose...")
    arm.inverse_kinematic_movement(place_pose)
    time.sleep(1)

    # 6. Open the gripper to release the cube.
    arm.node.get_logger().info("Releasing the cube (opening gripper)...")
    gripper.open()
    time.sleep(1)

    # 7. (Optional) Retreat after releasing by staying at the place pose.
    arm.node.get_logger().info("Retreating from the place pose...")
    arm.inverse_kinematic_movement(place_pose)
    time.sleep(1)

    arm.node.get_logger().info("Pick and place routine completed.")


def main():
    rclpy.init()
    # Initialize the arm and gripper interfaces.
    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()
    try:
        pick_and_place_over_obstacle(arm, gripper)
    except Exception as e:
        arm.node.get_logger().error(f"Error during pick and place: {e}")
    finally:
        # Shutdown ROS2 and clean up.
        rclpy.shutdown()
        gripper.shutdown()
        arm.shutdown()


if __name__ == '__main__':
    main()
