#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bringup = get_package_share_directory('food_del_robot_bringup')
    rplidar = get_package_share_directory('rplidar_ros')

    slam_params_path = os.path.join(
        bringup, 'config', 'real_robot', 'slam', 'slam_online_mapping.yaml'
    )

    rplidar_launch      = os.path.join(rplidar, 'launch', 'rplidar_a1_launch.py')
    display_launch      = os.path.join(bringup, 'launch', 'display.launch.py')
    hand_ctrl_launch    = os.path.join(bringup, 'launch', 'hand_controller.launch.py')

    return LaunchDescription([

        # Step 1 — IMU
        Node(
            package='yahboom_imu_10_axis',
            executable='imu_driver',
            name='imu_driver',
            output='screen',
        ),

        # Step 2 — LiDAR (via rplidar_a1_launch.py)
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(rplidar_launch)
                ),
            ]
        ),

        # Step 3 — Motor driver + odometry
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='zlac8015d_driver',
                    executable='drive_and_odom_v2',
                    name='drive_and_odom_v2',
                    output='screen',
                ),
            ]
        ),

        # Step 4 — Robot state publisher + RViz
        TimerAction(
            period=9.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(display_launch)
                ),
            ]
        ),

        # Step 5 — Hand controller
        TimerAction(
            period=12.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(hand_ctrl_launch)
                ),
            ]
        ),

        # Step 6 — SLAM Toolbox
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    parameters=[slam_params_path],
                    output='screen',
                ),
            ]
        ),
    ])