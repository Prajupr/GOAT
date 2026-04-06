#!/usr/bin/env python3
"""
Launch file for Butler Robot Delivery System with TurtleBot3
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate launch description for butler delivery with TurtleBot3"""
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    use_slam = LaunchConfiguration("use_slam")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_rviz = LaunchConfiguration("use_rviz")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false",
        description="Enable SLAM"
    )
    
    use_nav2_arg = DeclareLaunchArgument(
        "use_nav2",
        default_value="true",
        description="Enable Nav2 navigation"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Show RViz2"
    )

    use_auto_confirm_arg = DeclareLaunchArgument(
        "auto_confirm",
        default_value="false",
        description="Auto-confirm kitchen/table events for fast scenario testing"
    )

    use_auto_confirm_delay_arg = DeclareLaunchArgument(
        "auto_confirm_delay",
        default_value="0.5",
        description="Delay before auto confirmation in seconds"
    )

    # Set TurtleBot3 model explicitly
    turtlebot3_model = SetEnvironmentVariable(
        name="TURTLEBOT3_MODEL",
        value="waffle"
    )

    # TurtleBot3 Gazebo simulation with custom restaurant world
    world = os.path.join(
        get_package_share_directory("butler_delivery"),
        "worlds",
        "restaurant.world"
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("gazebo_ros"),
            "launch",
            "gazebo.launch.py"
        ),
        launch_arguments={
            "world": world,
        }.items()
    )

    # TurtleBot3 Navigation
    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("turtlebot3_navigation2"),
            "launch",
            "navigation2.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": os.path.join(
                get_package_share_directory("butler_delivery"),
                "config",
                "restaurant_nav2_params.yaml"
            ),
            "map": os.path.join(
                get_package_share_directory("butler_delivery"),
                "maps",
                "restaurant.yaml"
            ),
            "use_rviz": use_rviz,
        }.items(),
        condition=IfCondition(use_nav2)
    )

    # Robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'robot_state_publisher.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'True'
        }.items()
    )

    # Spawn TurtleBot3 robot
    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'turtlebot3_waffle',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # Butler Delivery Manager
    delivery_manager = Node(
        package="butler_delivery",
        executable="delivery_manager",
        name="delivery_manager",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"auto_confirm": LaunchConfiguration("auto_confirm")},
            {"auto_confirm_delay": LaunchConfiguration("auto_confirm_delay")},
        ],
    )

    # RViz visualization
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
            get_package_share_directory("butler_delivery"),
            "config",
            "butler_rviz.rviz"
        )],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz"))
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Show RViz2"
    )

    return LaunchDescription([
        use_slam_arg,
        use_nav2_arg,
        use_auto_confirm_arg,
        use_auto_confirm_delay_arg,
        use_rviz_arg,
        turtlebot3_model,
        gazebo,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        navigation,
        delivery_manager,
        rviz,
    ])
