#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    home = os.getenv('HOME')

    lifecycle_manager_config_path = os.path.join(
        home,
        'Food_Delivery_Robot/src/food_del_robot/config/Simulation/Lifecycle_manager/lifecycle_manager.yaml'
    )

    amcl_params = os.path.join(
        home,
        'Food_Delivery_Robot/src/food_del_robot/config/Simulation/AMCL/amcl_sim_config.yaml'
    )

    return LaunchDescription([

        # Launch map server after a small delay 
        TimerAction(
            period=3.0, 
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{
                        'yaml_filename': os.path.join(
                            home,
                            'Food_Delivery_Robot/src/food_del_robot/maps/Simulation_map/Simulation_map.yaml'
                        )
                    }]
                )
            ]
        ),

        # Launch AMCL after map server has started
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[amcl_params],  # added
                )
            ]
        ),

        # Launch lifecycle manager to handle node lifecycle states
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[lifecycle_manager_config_path],
        ),

    ])