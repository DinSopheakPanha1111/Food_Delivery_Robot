#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup = get_package_share_directory('food_del_robot_bringup')

    global_costmap_config = os.path.join(bringup, 'config', 'simulation', 'costmap', 'global_costmap_config.yaml')
    local_costmap_config = os.path.join(bringup, 'config', 'simulation', 'costmap', 'local_costmap_config.yaml')
    amcl_config_path = os.path.join(bringup, 'config', 'simulation', 'amcl', 'amcl_sim_config.yaml')
    map_yaml_path = os.path.join(bringup, 'maps', 'Simulation_map', 'Simulation_map.yaml')
    lifecycle_config_path = os.path.join(bringup, 'config', 'simulation', 'lifecycle_manager', 'lifecycle_manager.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'simulation', 'gazebo_in_restaurent_also_rviz.launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'hand_controller.launch.py'))
        ),
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
        Node(package='food_del_astar_planner', executable='astar_planner_node',
             name='astar_planner_node', output='screen',
             parameters=[global_costmap_config]),
        Node(package='dwb_controller', executable='main', name='dwb_controller', output='screen',
             parameters=[local_costmap_config]),
    ])
