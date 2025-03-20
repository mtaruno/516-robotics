from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kinova_package',
            executable='kinova_controller',
            name='kinova_arm_controller',
            output='screen'
        ),
    ])
