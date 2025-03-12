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
    time.sleep(2)  # Small delay between commands to ensure execution

def main():
    """Executes a sequence of ROS2 commands for motion and gripper control using custom control points."""
    
    commands = [
        # Step 1: Open Gripper (ensure it's open before picking)
        "ros2 run lab_quaternion control_gripper --ros-args -p command:=open",

        # Step 2: Move to Pick Pre-Pose (10 cm above the pick pose)
        # Pick Pre-Pose: x=0.463, y=-0.029, z=0.110+0.10=0.210
        # Orientation: (0.958, 0.013, -0.285, 0.007)
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.463 -p y:=-0.029 -p z:=0.210 -p qx:=0.958 -p qy:=0.013 -p qz:=-0.285 -p qw:=0.007",

        # Step 3: Move to Pick Pose
        # Pick Pose: x=0.463, y=-0.029, z=0.110; Orientation: (0.958, 0.013, -0.285, 0.007)
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.463 -p y:=-0.029 -p z:=0.110 -p qx:=0.958 -p qy:=0.013 -p qz:=-0.285 -p qw:=0.007",

        # Step 4: Close Gripper to grasp the cube
        "ros2 run lab_quaternion control_gripper --ros-args -p command:=close",

        # Step 5: Lift the cube by returning to Pick Pre-Pose (lifting straight up)
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.463 -p y:=-0.029 -p z:=0.210 -p qx:=0.958 -p qy:=0.013 -p qz:=-0.285 -p qw:=0.007",

        # Step 6: Move to Transfer Pose (over the obstacle)
        # Transfer Pose: x=0.439, y=0.111, z=0.501; Orientation: (0.709, 0.645, 0.271, 0.092)
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.439 -p y:=0.111 -p z:=0.501 -p qx:=0.709 -p qy:=0.645 -p qz:=0.271 -p qw:=0.092",

        # Step 7: Move to Place Pre-Pose (10 cm above the place pose)
        # Place Pre-Pose: x=0.094, y=0.411, z=0.120+0.10=0.220; Orientation: (-0.003, 0.999, -0.026, 0.028)
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.094 -p y:=0.411 -p z:=0.220 -p qx:=-0.003 -p qy:=0.999 -p qz:=-0.026 -p qw:=0.028",

        # Step 8: Move to Place Pose (to release the cube)
        # Place Pose: x=0.094, y=0.411, z=0.120; Orientation: (-0.003, 0.999, -0.026, 0.028)
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.094 -p y:=0.411 -p z:=0.120 -p qx:=-0.003 -p qy:=0.999 -p qz:=-0.026 -p qw:=0.028",

        # Step 9: Open Gripper to release the cube
        "ros2 run lab_quaternion control_gripper --ros-args -p command:=open",

        # Step 10 (Optional): Retreat back to Place Pre-Pose
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.094 -p y:=0.411 -p z:=0.220 -p qx:=-0.003 -p qy:=0.999 -p qz:=-0.026 -p qw:=0.028",
    ]
    
    for command in commands:
        run_ros2_command(command)

if __name__ == "__main__":
    main()
