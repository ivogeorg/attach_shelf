from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    final_approach_node = Node(
        package='attach_shelf',
        executable='approach_shelf_service_server_node',
        output='screen',
        name='approach_shelf_service_server_node',
        emulate_tty=True,
    )

    # create and return launch description object
    return LaunchDescription(
        [
            final_approach_node
        ]
    )