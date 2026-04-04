#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup = get_package_share_directory('food_del_robot_bringup')

    smac_sim_config = os.path.join(bringup, 'config', 'simulation', 'planner', 'smac_sim_config.yaml')
    local_costmap_config = os.path.join(bringup, 'config', 'simulation', 'costmap', 'local_costmap_config.yaml')
    lifecycle_planner_config = os.path.join(bringup, 'config', 'simulation', 'lifecycle_manager', 'lifecycle_manager_planner.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'simulation', 'gazebo_in_restaurent_also_rviz.launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'simulation', 'amcl_sim.launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'hand_controller.launch.py'))
        ),
        Node(package='dwb_controller', executable='main', name='dwb_controller', output='screen',
             parameters=[local_costmap_config, {'use_sim_time': True}]),
        Node(package='food_del_goal_bridge', executable='path_recorder', name='path_recorder', output='screen',
             parameters=[{'use_sim_time': True}]),
        TimerAction(period=6.0, actions=[
            Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
                 parameters=[smac_sim_config, {'use_sim_time': True}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_planner', output='screen',
                 parameters=[lifecycle_planner_config]),
            Node(package='food_del_goal_bridge', executable='goal_bridge', name='goal_bridge', output='screen',
                 parameters=[{'use_sim_time': True}]),
        ]),
    ])
