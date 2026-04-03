#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():

    home = os.getenv('HOME')
    src  = os.path.join(home, 'Food_Delivery_Robot', 'src')

    # ── Paths (all from src — no install/ dependency) ─────────────────────────
    global_costmap_config  = os.path.join(src, 'food_del_robot', 'config', 'Simulation', 'Costmap', 'global_costmap_config.yaml')
    local_costmap_config   = os.path.join(src, 'food_del_robot', 'config', 'Simulation', 'Costmap', 'local_costmap_config.yaml')
    amcl_config_path       = os.path.join(src, 'food_del_robot', 'config', 'Simulation', 'AMCL', 'amcl_sim_config.yaml')
    map_yaml_path          = os.path.join(src, 'food_del_robot', 'maps', 'Simulation_map', 'Simulation_map.yaml')
    lifecycle_config_path  = os.path.join(src, 'food_del_robot', 'config', 'Simulation', 'Lifecycle_manager', 'lifecycle_manager.yaml')

    hand_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(src, 'food_del_robot_description', 'launch', 'Real_Robot', 'Hand_Controller', 'hand_controller.launch.py')
        )
    )

    return LaunchDescription([

        # ── Gazebo (robot model + world + RViz) ───────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(src, 'food_del_robot_description', 'launch', 'Simulation', 'Gazebo', 'robot_gazebo.launch.py')
            )
        ),

        # ── Hand controller ───────────────────────────────────────────────────
        hand_controller,

        # ── Map server (after 3 s) ────────────────────────────────────────────
        TimerAction(period=3.0, actions=[
            Node(package='nav2_map_server', executable='map_server',
                 name='map_server', output='screen',
                 parameters=[{'yaml_filename': map_yaml_path}]),
        ]),

        # ── AMCL (after 3 s) ─────────────────────────────────────────────────
        TimerAction(period=3.0, actions=[
            Node(package='nav2_amcl', executable='amcl',
                 name='amcl', output='screen',
                 parameters=[amcl_config_path]),
        ]),

        # ── Lifecycle manager ─────────────────────────────────────────────────
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager', output='screen',
             parameters=[lifecycle_config_path]),

        # ── A* planner ────────────────────────────────────────────────────────
        Node(package='food_del_astar_planner', executable='astar_planner_node',
             name='astar_planner_node', output='screen',
             parameters=[global_costmap_config]),

        # ── DWB controller ────────────────────────────────────────────────────
        Node(package='dwb_controller', executable='main',
             name='dwb_controller', output='screen',
             parameters=[local_costmap_config]),
    ])
