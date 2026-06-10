#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup = get_package_share_directory('food_del_robot_bringup')

    map_yaml_path = os.path.join(bringup, 'maps', 'Simulation_map', 'Simulation_map.yaml')
    lifecycle_config_path = os.path.join(
        bringup, 'config', 'simulation', 'lifecycle_manager', 'lifecycle_manager_map_only.yaml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'simulation', 'gazebo_in_restaurent_also_rviz.launch.py')
            )
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[lifecycle_config_path],
        ),
        TimerAction(period=3.0, actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'yaml_filename': map_yaml_path, 'use_sim_time': True}],
            ),
        ]),
    ])
