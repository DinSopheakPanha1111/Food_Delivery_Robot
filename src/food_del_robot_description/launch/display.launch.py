#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.substitutions import Command

def generate_launch_description():
    urdf_path = os.path.join(
        os.getenv('HOME'),
        'food_del_robot/src/food_del_robot_description/urdf/my_main.urdf.xacro'
    )

    rviz_config_path = os.path.join(
        os.getenv('HOME'),
        'food_del_robot/src/food_del_robot_description/rviz/food_del_robot.rviz'
    )

    return LaunchDescription([
        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path])
            }]
        ),

        # RViz2 (with config)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
    ])
