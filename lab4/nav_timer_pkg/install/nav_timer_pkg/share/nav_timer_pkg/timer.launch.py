from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav_pose_timer_pkg',
            executable='nav_pose_timer',
            name='nav_pose_timer',
            output='screen',
            parameters=[
                # You can add parameters here
                {'use_astar': True}  # Example parameter
            ]
        )
    ])