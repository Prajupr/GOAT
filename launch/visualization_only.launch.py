#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('butler_delivery')

    return LaunchDescription([
        # Delivery manager node
        Node(
            package='butler_delivery',
            executable='delivery_manager',
            name='delivery_manager',
            output='screen',
            parameters=[{
                'auto_confirm': LaunchConfiguration('auto_confirm', default='false'),
                'auto_confirm_delay': LaunchConfiguration('auto_confirm_delay', default='1.0'),
            }]
        ),

        # Static TF publishers for locations
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_home',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'home']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_kitchen',
            arguments=['1.0', '0', '0', '0', '0', '0', 'world', 'kitchen']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_table1',
            arguments=['-0.5', '0.5', '0', '0', '0', '0', 'world', 'table1']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_table2',
            arguments=['-0.5', '-0.5', '0', '0', '0', '0', 'world', 'table2']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_table3',
            arguments=['1.5', '-0.5', '0', '0', '0', '0', 'world', 'table3']
        ),

        # Robot visualizer (text-based)
        Node(
            package='butler_delivery',
            executable='robot_visualizer',
            name='robot_visualizer',
            output='screen'
        ),
    ])