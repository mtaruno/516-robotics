import numpy as np
from pyquaternion import Quaternion
from .gen3lite_pymoveit2 import Gen3LiteArm
from geometry_msgs.msg import Pose
import rclpy

def main(args=None):
    # -------------------------------
    # Step 1: Load position data from CSV
    # -------------------------------
    data_path = 'data.csv'
    try:
        # Load three columns: x, y, z
        data = np.loadtxt(data_path, delimiter=',')
    except Exception as e:
        print(f"Error loading CSV: {e}")
        return

    # Ensure the file is not empty
    if data.shape[0] == 0:
        print("Error: data.csv is empty!")
        return
    
    # -------------------------------
    # Step 2: Define Start and End Orientations
    # -------------------------------
    # Define the start (identity rotation) and end orientations.
    qStart = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)  # Start pose
    qEnd   = Quaternion(w=0.0, x=0.0, y=1.0, z=0.0)  # End pose
    
    # -------------------------------
    # Step 3: Compute Quaternion Interpolation
    # -------------------------------
    num_positions = len(data)
    if num_positions < 2:
        print("Error: Not enough positions for interpolation!")
        return

    # To obtain exactly 'num_positions' orientations, interpolate (num_positions - 2) inner quaternions,
    # then add qStart and qEnd at the beginning and end.
    q_interpolated = list(Quaternion.intermediates(qStart, qEnd, num_positions - 2, include_endpoints=True))
    qList_elements = [qStart.elements] + [q.elements for q in q_interpolated] + [qEnd.elements]

    # -------------------------------
    # Step 4: Initialize ROS2 and the Arm
    # -------------------------------
    rclpy.init()
    arm = Gen3LiteArm()
    
    # -------------------------------
    # Step 5: Command the Arm Along the Trajectory
    # -------------------------------
    for i in range(num_positions):
        pose = Pose()
        # Set position (x, y, z) from CSV
        pose.position.x = float(data[i][0])
        pose.position.y = float(data[i][1])
        pose.position.z = float(data[i][2])

        # Get corresponding quaternion elements.
        # pyquaternion returns [w, x, y, z], but ROS expects (x, y, z, w).
        q_elem = qList_elements[i]
        pose.orientation.x = float(q_elem[1])
        pose.orientation.y = float(q_elem[2])
        pose.orientation.z = float(q_elem[3])
        pose.orientation.w = float(q_elem[0])

        # Debug print to verify pose values
        print(f"Pose {i}: ({pose.position.x}, {pose.position.y}, {pose.position.z})")
        print(f"Orientation {i}: ({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})")
        print(f"Moving to Pose {i}...")
        
        # Command the arm to move using inverse kinematics
        arm.inverse_kinematic_movement(pose)
    
    # -------------------------------
    # Step 6: Shutdown
    # -------------------------------
    arm.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

