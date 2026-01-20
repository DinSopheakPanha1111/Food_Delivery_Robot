#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_food_del_robot = get_package_share_directory('food_del_robot_description')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_food_del_robot, 'launch',
                         'robot.launch.py'),
        )
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_food_del_robot, 'launch',
                         'joystick.launch.py'),
        )
    )

    zlac_driver = Node(
        package='zlac8015d_driver',
        executable='drive_and_odom',
        name='zlac8015d_driver',         
        output='screen',                 
    )

    return LaunchDescription([
        robot,
        joystick,
        zlac_driver
    ])
