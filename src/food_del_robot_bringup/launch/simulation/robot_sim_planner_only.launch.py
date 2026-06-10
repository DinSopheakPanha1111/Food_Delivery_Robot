#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup = get_package_share_directory('food_del_robot_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    smac_sim_config = os.path.join(
        bringup, 'config', 'simulation', 'planner', 'smac_sim_config.yaml'
    )

    map_yaml_path = os.path.join(
        bringup, 'maps', 'Simulation_map', 'Simulation_map.yaml'
    )

    amcl_config_path = os.path.join(
        bringup, 'config', 'simulation', 'amcl', 'amcl_sim_config.yaml'
    )

    lifecycle_config_path = os.path.join(
        bringup, 'config', 'simulation', 'lifecycle_manager', 'lifecycle_manager_planner_only.yaml'
    )

    keepout_filter_params = os.path.join(
        bringup,
        'config',
        'simulation',
        'map_filter',
        'keep_out_filter_config.yaml'
    )

    keepout_mask_yaml = os.path.join(
        bringup,
        'maps',
        'Simulation_map',
        'keepout_mask.yaml'
    )

    return LaunchDescription([

        # =========================
        # Gazebo + RViz
        # =========================
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

        # =========================
        # MAP SERVER
        # =========================
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {
                    'yaml_filename': map_yaml_path,
                    'use_sim_time': True
                }
            ]
        ),

        # =========================
        # LOCALIZATION
        # =========================
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                amcl_config_path,
                {'use_sim_time': True}
            ]
        ),

        # =========================
        # PLANNER SERVER
        # =========================
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                smac_sim_config,
                {'use_sim_time': True}
            ]
        ),

        # =========================
        # KEEP OUT FILTER SERVERS
        # =========================
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='keepout_filter_mask_server',
                    output='screen',
                    parameters=[
                        keepout_filter_params,
                        {
                            'use_sim_time': True,
                            'yaml_filename': keepout_mask_yaml
                        }
                    ]
                ),
                Node(
                    package='nav2_map_server',
                    executable='costmap_filter_info_server',
                    name='keepout_costmap_filter_info_server',
                    output='screen',
                    parameters=[
                        keepout_filter_params,
                        {'use_sim_time': True}
                    ]
                ),
            ]
        ),

        # =========================
        # LIFECYCLE MANAGER
        # =========================
        TimerAction(
            period=8.0,
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

        # =========================
        # Depth → PointCloud2
        # =========================
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
                        ('image_rect', '/camera/depth/image_raw'),
                        ('camera_info', '/camera/camera_info'),
                        ('points', '/camera/depth/points'),
                    ]
                ),
            ]
        ),
    ])
