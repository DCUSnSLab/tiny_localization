#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('tiny_localization')
    
    # Launch arguments
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='tiny_localization_node',
        description='Node name'
    )
    
    node_namespace_arg = DeclareLaunchArgument(
        'node_namespace',
        default_value='localization',
        description='Node namespace'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('tiny_localization'),
            'config',
            'config.yaml'
        ]),
        description='Path to config file'
    )
    
    tf_broadcast_enabled_arg = DeclareLaunchArgument(
        'tf_broadcast_enabled',
        default_value='true',
        description='TF broadcast enable flag'
    )
    
    # Main localization node
    localization_node = Node(
        package='tiny_localization',
        executable='tiny_localization_node',
        name=LaunchConfiguration('node_name'),
        namespace=LaunchConfiguration('node_namespace'),
        parameters=[LaunchConfiguration('config_file'), {'use_sim_time': True}],
        output='screen'
    )
    
    # TF broadcast nodes group
    tf_broadcast_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('tf_broadcast_enabled')),
        actions=[
            # odom_utm -> base_link TF broadcast
            Node(
                package='tiny_localization',
                executable='odom_frame_broadcast.py',
                name='odom_frame_broadcaster',
                output='screen'
            ),
            
            # gps_utm -> odom_utm TF broadcast
            Node(
                package='tiny_localization',
                executable='gps_frame_broadcast.py',
                name='gps_frame_broadcaster',
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        node_name_arg,
        node_namespace_arg,
        config_file_arg,
        tf_broadcast_enabled_arg,
        localization_node,
        tf_broadcast_group
    ])