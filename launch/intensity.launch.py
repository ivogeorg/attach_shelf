from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node (
            package='attach_shelf',
            executable='intensity_poker_node',
            output='screen'
        ),
    ])