#!/bin/bash

# Source your ROS 2 workspace 
source ~/ros2_ws/install/setup.bash

# Launch TurtleBot script
echo "turtlebot go brrrrrrrrrrrrr..."
ros2 launch turtle basket

# Wait until TurtleBot is fully initialized
sleep 5  # Adjust if needed

# Launch Kinova arm script
echo "arm coming!!!!!!!!..."
ros2 launch <kinova_package> <kinova_launch_file>.launch.py &

# Keep script running (optional)
wait $TURTLEBOT_PID
