from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node (
            package='attach_shelf',
            executable='approach_shelf_service_server_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'final_approach':True}
            ]
        ),
        Node (
            package='attach_shelf',
            executable='pre_approach_v2_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'obstacle':0.3},
                {'degrees':-90.0}
            ]
        ),
    ])


