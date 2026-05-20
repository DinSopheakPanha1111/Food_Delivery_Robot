#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup = get_package_share_directory('food_del_robot_bringup')

    # Path to included launch file
    hand_controller_launch = os.path.expanduser(
        '~/Food_Delivery_Robot/src/food_del_robot_bringup/launch/hand_controller.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hand_controller_launch)
        ),

        Node(
            package='zlac8015d_driver',
            executable='drive_and_odom_v2',
            name='drive_and_odom_v2'
        ),
    ])