#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    joy_params = os.path.join(get_package_share_directory('food_del_robot_description'), 'config', 'joystick.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
    )

    cmd_vel_to_twist_stamped_node = Node(
        package='zlac8015d_driver',
        executable='cmd_vel_to_twist_stamped',
        name='cmd_vel_to_twist_stamped',
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        cmd_vel_to_twist_stamped_node
    ])
