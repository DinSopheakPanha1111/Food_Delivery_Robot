#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    desc = get_package_share_directory('food_del_robot_description')
    bringup = get_package_share_directory('food_del_robot_bringup')

    urdf_path = os.path.join(desc, 'urdf', 'food_del_robot.urdf.xacro')
    rviz_config_path = os.path.join(desc, 'rviz', 'just_robot.rviz')
    ekf_config_path = os.path.join(bringup, 'config', 'real_robot', 'ekf', 'ekf_config.yaml')
    rplidar_launch_path = os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
    hand_controller_launch = os.path.join(bringup, 'launch', 'hand_controller.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(rplidar_launch_path)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(hand_controller_launch)),
        ExecuteProcess(
            cmd=['ros2', 'run', 'robot_localization', 'ekf_node',
                 '--ros-args', '--params-file', ekf_config_path],
            output='screen',
        ),
        Node(package='yahboom_imu_10_axis', executable='imu_driver', name='imu_driver', output='screen'),
        Node(package='zlac8015d_driver', executable='drive_and_odom', name='drive_and_odom', output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
        ),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config_path], output='screen'),
    ])
