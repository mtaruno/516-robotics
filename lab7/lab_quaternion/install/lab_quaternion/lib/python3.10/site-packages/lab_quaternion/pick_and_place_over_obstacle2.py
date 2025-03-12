#!/usr/bin/env python3
import os
import time

def run_ros2_command(command):
    """Executes a ROS2 command and waits for completion."""
    print(f"\n🚀 Executing: {command}")
    exit_code = os.system(command)
    if exit_code != 0:
        print(f"⚠️ Command failed: {command}")
    else:
        print(f"✅ Completed: {command}")
    time.sleep(2)  # Delay between commands for safe execution

def main():
    """
    This script executes a simple pick-and-place sequence using three key positions:
      1. Start (pick) pose
      2. Transfer pose (to clear obstacles)
      3. End (place) pose
    It also commands the gripper to open before picking and to close to grasp and then open to release.
    """

    commands = [
        # Step 1: Ensure the gripper is open
        "ros2 run lab_quaternion control_gripper --ros-args -p command:=open",
        
        # Step 2: Move to the Start (Pick) Pose
        # Start Pose: x=0.257, y=-0.094, z=0.103, Orientation: (qx=0.076, qy=0.977, qz=-0.042, qw=0.194)
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.257 -p y:=-0.094 -p z:=0.103 -p qx:=0.076 -p qy:=0.977 -p qz:=-0.042 -p qw:=0.194",
        
        # Step 3: Close the gripper to grasp the object at the start pose
        "ros2 run lab_quaternion control_gripper --ros-args -p command:=close",
        
        # Step 4: Move to the Transfer Pose (clearing an obstacle)
        # Transfer Pose: x=0.439, y=0.111, z=0.501, Orientation: (qx=0.709, qy=0.645, qz=0.271, qw=0.092)
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.439 -p y:=0.111 -p z:=0.501 -p qx:=0.709 -p qy:=0.645 -p qz:=0.271 -p qw:=0.092",
        
        # Step 5: Move to the End (Place) Pose
        # End Pose: x=0.374, y=-0.378, z=0.102, Orientation: (qx=0.063, qy=0.988, qz=-0.041, qw=0.134)
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.374 -p y:=-0.378 -p z:=0.102 -p qx:=0.063 -p qy:=0.988 -p qz:=-0.041 -p qw:=0.134",
        
        # Step 6: Open the gripper to release the object at the end pose
        "ros2 run lab_quaternion control_gripper --ros-args -p command:=open"
    ]
    
    for command in commands:
        run_ros2_command(command)

if __name__ == "__main__":
    main()
