#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    handcontroller_launch_path = os.path.join(get_package_share_directory('food_del_robot'), 'launch', 'Real_Robot','Hand_Controller','hand_controller.launch.py')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[handcontroller_launch_path],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[handcontroller_launch_path],
    )

    drive_node = Node(
        package='zlac8015d_driver',
        executable='drive_and_odom_v2',
        name='drive_and_odom_v2',
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        drive_node,
    ])
