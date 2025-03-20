
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch nav2_bringup bringup_launch.py map:=./lab2/map.yaml
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz

nano /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml

# Terminal 1: Launch your robot (simulation or real)
# ros2 launch your_robot_package robot_bringup.launch.py
# Terminal 2: Launch navigation 
# ros2 launch nav2_bringup navigation_launch.py


colcon build --packages-select nav_timer_pkg

source install/setup.bash

ros2 run nav_timer_pkg nav_timer

ros2 param set /planner_server GridBased.use_astar true

ros2 param get /planner_server GridBased.use_astar # for diagnosis

