#!/usr/bin/python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'attach_shelf'
    pkg_share_name = FindPackageShare(pkg_name)

    config_dir_name = 'config'

    # ========
    # ARGUMENT
    # ========
    # use_sim_time argument (determines simulator vs real lab)
    use_sim_time_f = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time', default_value=TextSubstitution(text='true'),
            description='config for simulator (true) or lab (false)')

    # ====
    # NODE
    # ====
    # cart approach test node
    cart_front_midpoint_test_node = Node(
        package='attach_shelf', executable='cart_front_midpoint_test_node',
        output='screen', name='cart_front_midpoint_test_node', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time_f}])

    # ==================
    # LAUNCH DESCRIPTION
    # ==================
    return LaunchDescription(
        [
            use_sim_time_arg,
            cart_front_midpoint_test_node
        ]
    )