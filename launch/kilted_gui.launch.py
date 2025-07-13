from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kilted_gui',
            executable='kilted_gui',
            name='kilted_gui',
            output='screen'
        )
    ])
