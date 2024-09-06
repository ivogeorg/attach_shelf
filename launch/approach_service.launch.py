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
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=TextSubstitution(text='true'))

    # ======
    # CONFIG
    # ======
    # attach service server node configuration
    attach_service_config_file_name_sim = 'approach_service_server_node_sim.yaml'
    attach_service_config_path_sim = PathJoinSubstitution([pkg_share_name, config_dir_name, attach_service_config_file_name_sim])

    attach_service_config_file_name_lab = 'approach_service_server_node_lab.yaml'
    attach_service_config_path_lab = PathJoinSubstitution([pkg_share_name, config_dir_name, attach_service_config_file_name_lab])

    # ====
    # NODE
    # ====
    # `/approach_shelf` service server node
    final_approach_node_sim = Node(
        package='attach_shelf', executable='approach_shelf_service_server_node',
        output='screen', name='approach_shelf_service_server_node', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time_f}, attach_service_config_path_sim],
        remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')],
        condition=IfCondition(use_sim_time_f)
    )

    final_approach_node_lab = Node(
        package='attach_shelf', executable='approach_shelf_service_server_node',
        output='screen', name='approach_shelf_service_server_node', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time_f}, attach_service_config_path_lab],
        condition=UnlessCondition(use_sim_time_f)
    )

    # ==================
    # LAUNCH DESCRIPTION
    # ==================
    return LaunchDescription(
        [
            use_sim_time_arg,
            final_approach_node_sim,
            final_approach_node_lab
        ]
    )