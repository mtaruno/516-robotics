#!/usr/bin/env python3
import time
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from pyquaternion import Quaternion as PyQuaternion
# Import your arm and gripper control classes (update these import paths as needed)
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper  
import numpy as np


# green_cube_pick_data = np.loadtxt('green_cube_pick_data.csv', delimiter=',')
# green_cube_put_data = np.loadtxt('green_cube_put_data.csv', delimiter=',')
# red_cube_pick_data = np.loadtxt('red_cube_pick_data.csv', delimiter=',')



def slerp(data, qStart, qEnd, arm):
    qList = []
    for q in PyQuaternion.intermediates(qStart, qEnd, len(data)-2, include_endpoints=True):
        qList.append(q.elements)
    for i in range(len(data)):
        pose = Pose()
        pose.position.x = data[i][0]
        pose.position.y = data[i][1]
        pose.position.z = data[i][2]
        pose.orientation.x = qList[i][0]
        pose.orientation.y = qList[i][1]
        pose.orientation.z = qList[i][2]
        pose.orientation.w = qList[i][3]
        arm.inverse_kinematic_movement(pose)
        print(f"Reached control point {i}")

def pick_and_place(pick_data, put_data, arm, gripper, qStart, qPick, qPut):
    gripper.open()
    slerp(pick_data, qStart, qPick, arm)
    time.sleep(0.5)
    print("Got to pick position")
    gripper.move_to_position(0.48)
    #arm.inverse_kinematic_movement(put)
    time.sleep(0.5)
    slerp(put_data, qPick, qPut, arm)
    #arm.inverse_kinematic_movement(put_pose)
    print("Got to put position")
    gripper.open()

def main():
    rclpy.init()
    # Initialize arm and gripper interfaces
    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()
    qStartGreenCube = PyQuaternion(array=np.array([0,0,0.0,1]))
    qPickGreenCube = PyQuaternion(array=np.array([0,0.842,0,0.540]))
    qPutGreenCube = PyQuaternion(array=np.array([0,0.706, 0.0, 0.708]))
    
    
    pick_and_place(green_cube_pick_data, green_cube_put_data, arm, gripper, qStartGreenCube, qPickGreenCube, qPutGreenCube)
    
    #qPickRedCube = PyQuaternion(array=np.array([0.03,0.841,-0.024,0.540]))
    #qPutRedCube = PyQuaternion(array=np.array([0.0, 0.851, 0.0, 0.525]))
    #pick_and_place(red_cube_pick_data, red_cube_put_data, arm, gripper,qStartGreenCube, qPickRedCube, qPutRedCube)

    # Shutdown ROS2 and clean up
    rclpy.shutdown()
    gripper.shutdown()
    arm.shutdown()

if __name__ == '__main__':
    main()


