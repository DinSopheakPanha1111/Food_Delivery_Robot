#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('food_del_robot_description')

    robot_gazebo = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'robot_gazebo.launch.xml')
        )
    )

    amcl_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'amcl_localization.launch.py')
        )
    )

    return LaunchDescription([
        robot_gazebo,
        amcl_localization,
    ])