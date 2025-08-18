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
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') # 시뮬레이션 환경인 경우 true, 밖이면 false

    map_file = PathJoinSubstitution([
                    FindPackageShare('gmserver'),
                    'maps',
                    'zzzzz.json'
                ])

    # Package directory
    pkg_dir = get_package_share_directory('tiny_localization')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Param for use_sim_time'
    )
    
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
    
    map_file_path_arg = DeclareLaunchArgument(
        'map_file_path',
        default_value=map_file,
        description='Path to map JSON file for map frame broadcast'
    )
    
    # Main localization node
    localization_node = Node(
        package='tiny_localization',
        executable='tiny_localization_node',
        name=LaunchConfiguration('node_name'),
        namespace=LaunchConfiguration('node_namespace'),
        parameters=[LaunchConfiguration('config_file'), {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # TF broadcast nodes group
    tf_broadcast_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('tf_broadcast_enabled')),
        actions=[
            # odom -> base_link TF broadcast
            Node(
                package='tiny_localization',
                executable='odom_frame_broadcast.py',
                name='odom_frame_broadcaster',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            
            # map -> odom TF broadcast
            # Node(
            #     package='tiny_localization',
            #     executable='gps_frame_broadcast.py',
            #     name='gps_frame_broadcaster',
            #     output='screen',
            #     parameters=[{
            #         'map_file_path': LaunchConfiguration('map_file_path'),
            #         'use_sim_time': use_sim_time
            #     }]
            # )
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        node_name_arg,
        node_namespace_arg,
        config_file_arg,
        tf_broadcast_enabled_arg,
        map_file_path_arg,
        localization_node,
        tf_broadcast_group
    ])