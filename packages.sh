

# utilized to build all packages within a ROS2 workspace
colcon build --packages-select lab_quaternion
source install/setup.bash

ros2 run lab_quaternion trajectory_from_csv



# colcon is a CL tool designe dto improve the workflow of building, testing, and using multiple software packages in ROS2


ros2 launch lab_quaternion sort_world


ros2 launch lab_quaternion sort_world.launch.py \
robot_type:=gen3_lite \
robot_name:=gen3_lite \
dof:=6 \
gripper:=gen3_lite_2f \
launch_rviz:=false