#!/usr/bin/env python3
"""
Minimal launch file to start Gazebo and spawn TurtleBot3 in the restaurant world.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('butler_delivery')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')

    world_file = os.path.join(pkg_dir, 'worlds', 'restaurant.world')

    use_rviz = LaunchConfiguration('use_rviz', default='false')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    declare_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Whether to launch RViz2')

    declare_x_pose_arg = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Initial x pose for TurtleBot3 spawn')

    declare_y_pose_arg = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Initial y pose for TurtleBot3 spawn')

    # Use TurtleBot3 waffle by default.
    turtlebot3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='waffle'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
        }.items()
    )

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_share, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items()
    )

    return LaunchDescription([
        declare_rviz_arg,
        declare_x_pose_arg,
        declare_y_pose_arg,
        turtlebot3_model,
        gazebo,
        spawn_turtlebot,
    ])
