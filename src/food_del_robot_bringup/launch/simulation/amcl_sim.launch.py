#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup = get_package_share_directory('food_del_robot_bringup')

    map_yaml_path = os.path.join(bringup, 'maps', 'Simulation_map', 'Simulation_map.yaml')
    amcl_config_path = os.path.join(bringup, 'config', 'simulation', 'amcl', 'amcl_sim_config.yaml')
    lifecycle_config_path = os.path.join(bringup, 'config', 'simulation', 'lifecycle_manager', 'lifecycle_manager.yaml')

    return LaunchDescription([
        TimerAction(period=3.0, actions=[
            Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
                 parameters=[{'yaml_filename': map_yaml_path}]),
        ]),
        TimerAction(period=3.0, actions=[
            Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
                 parameters=[amcl_config_path]),
        ]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager', output='screen',
             parameters=[lifecycle_config_path]),
    ])
