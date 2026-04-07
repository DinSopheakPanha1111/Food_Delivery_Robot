#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup = get_package_share_directory('food_del_robot_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    smac_sim_config = os.path.join(
        bringup, 'config', 'simulation', 'planner', 'smac_sim_config.yaml'
    )
    dwb_sim_config = os.path.join(
        bringup, 'config', 'simulation', 'controller', 'dwb_sim_config.yaml'
    )
    bt_navigator_config = os.path.join(
        bringup, 'config', 'simulation', 'bt_navigator', 'bt_navigator_sim_config.yaml'
    )
    map_yaml_path = os.path.join(
        bringup, 'maps', 'Simulation_map', 'Simulation_map.yaml'
    )
    amcl_config_path = os.path.join(
        bringup, 'config', 'simulation', 'amcl', 'amcl_sim_config.yaml'
    )
    lifecycle_config_path = os.path.join(
        bringup, 'config', 'simulation', 'lifecycle_manager', 'lifecycle_manager_sim.yaml'
    )

    return LaunchDescription([

        # Gazebo + RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    bringup,
                    'launch',
                    'simulation',
                    'gazebo_in_restaurent_also_rviz.launch.py'
                )
            )
        ),

        # Hand controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    bringup,
                    'launch',
                    'hand_controller.launch.py'
                )
            )
        ),

        # Depth image → PointCloud2 for STVL obstacle detection
        # Waits 8s to ensure Gazebo camera is fully publishing first
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='depth_image_proc',
                    executable='point_cloud_xyz_node',
                    name='depth_to_pointcloud',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    remappings=[
                        ('image_rect',  '/camera/depth/image_raw'),
                        ('camera_info', '/camera/camera_info'),
                        ('points',      '/camera/depth/points'),
                    ]
                ),
            ]
        ),

        # All Nav2 nodes start immediately
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_path}, {'use_sim_time': True}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config_path, {'use_sim_time': True}]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[smac_sim_config, {'use_sim_time': True}]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[dwb_sim_config, {'use_sim_time': True}]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_config, {'use_sim_time': True}]
        ),

        # Single lifecycle manager — delayed 5s so all 5 nodes are up first
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager',
                    output='screen',
                    parameters=[lifecycle_config_path]
                ),
            ]
        ),

        Node(
            package='food_del_goal_bridge',
            executable='path_recorder',
            name='path_recorder',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # goal_bridge delayed 10s to ensure nav stack is fully active
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='food_del_goal_bridge',
                    executable='goal_bridge',
                    name='goal_bridge',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                ),
            ]
        ),
    ])