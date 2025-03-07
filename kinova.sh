# kinova arm commands

# Connect your laptop's ROS environment to robot arm
ros2 launch kortex_bringup gen3_lite.launch.py \
robot_ip:=10.18.2.230\
launch_rviz:=false

# Launch moveit's planning server and RViz
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py \
robot_ip:=10.18.2.230

# Launch moveit in the simulation
ros2 launch kinova_gen3_lite_moveit_config sim.launch.py 

# Print out the current transform (position and orientation) between base_link and end_effector_link
ros2 run tf2_ros tf2_echo base_link end_effector_link


# list the available TF frames - generates TF tree showing all connected frames
ros2 run tf2_tools view_frames

# graphically view tf tree
ros2 run rqt_tf_tree rqt_tf_tree 






