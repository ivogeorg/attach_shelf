from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Package
    ros2_pkg = "attach_shelf"

    # DeclareLaunchArgument
    obstacle_arg = DeclareLaunchArgument("obstacle", default_value=TextSubstitution(text="0.45"))
    degrees_arg = DeclareLaunchArgument("degrees", default_value=TextSubstitution(text="-90.0"))
    final_approach_arg = DeclareLaunchArgument("final_approach", default_value=TextSubstitution(text="false"))

    # LaunchConfiguration
    obstacle_f = LaunchConfiguration('obstacle')
    degrees_f = LaunchConfiguration('degrees')
    final_approach_f = LaunchConfiguration('final_approach')

    # IncludeLaunchDescription
    approach_service_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(ros2_pkg),
                'launch',
                'approach_service.launch.py'
            ])
        ])
    )
    pre_approach_v2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(ros2_pkg),
                'launch',
                'pre_approach_v2.launch.py'
            ])  
        ]),
        launch_arguments= {
            'obstacle' : obstacle_f,
            'degrees' : degrees_f,
            'final_approach' : final_approach_f
        }.items()
    )

    # LaunchDescription
    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        approach_service_launch,
        pre_approach_v2_launch
    ])


