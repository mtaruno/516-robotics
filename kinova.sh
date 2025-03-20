# kinova arm commands


# connectivity
fastdds

# make sure you have sourced the correct .bashrc file
source /opt/ros/humble/setup.bash
source ~/TECHIN516/kinova_ws/install/setup.bash
source /usr/share/gazebo/setup.bash
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=10
export ROS_DISCOVERY_SERVER=10.18.3.90:11811
export RMW_IMPLEMENTATION='rmw_fastrtps_cpp'

# Connect your laptop's ROS environment to robot arm
ros2 launch kortex_bringup gen3_lite.launch.py \
robot_ip:=10.18.2.230\
launch_rviz:=false


ros2 launch kinova_gen3_lite_moveit_config sim.launch.py


# Launch moveit's planning server and RViz - this is a script that runs a Python launch script in the background that setes up and starts multiple ROS2 nodes
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py \
robot_ip:=10.18.2.240



# ros2 launch is a ROS2 tool for launching multiple nodes together - looks for gen3_lite.launch.py inside kortex_bringup package

ros2 node list 



# Launch moveit in the simulation
ros2 launch kinova_gen3_lite_moveit_config sim.launch.py 

# Print out the current transform (position and orientation) between base_link and end_effector_link
ros2 run tf2_ros tf2_echo base_link end_effector_link


# list the available TF frames - generates TF tree showing all connected frames
ros2 run tf2_tools view_frames

# graphically view tf tree
ros2 run rqt_tf_tree rqt_tf_tree 






