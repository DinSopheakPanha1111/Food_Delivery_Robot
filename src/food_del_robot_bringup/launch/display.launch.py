#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    desc = get_package_share_directory('food_del_robot_description')
    xacro_file = os.path.join(desc, 'urdf', 'food_del_robot.urdf.xacro')
    rviz_config_path = os.path.join(desc, 'rviz', 'food_del_robot_slam.rviz')
    robot_description = xacro.process_file(xacro_file, mappings={'use_sim': 'true'}).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            name='rviz2',
        ),
    ])
