from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    final_approach_arg = DeclareLaunchArgument(
        "final_approach", default_value="false"
    )

    final_approach_f = LaunchConfiguration('final_approach')

    final_approach_node = Node(
        package='attach_shelf',
        executable='approach_shelf_service_server_node',
        output='screen',
        name='approach_shelf_service_server_node',
        emulate_tty=True,
        arguments=["-final_approach", final_approach_f,
                   ]
    )

    # create and return launch description object
    return LaunchDescription(
        [
            final_approach_arg,
            final_approach_node
        ]
    )