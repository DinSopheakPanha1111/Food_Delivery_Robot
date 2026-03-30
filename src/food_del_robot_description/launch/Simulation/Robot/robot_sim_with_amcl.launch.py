#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('food_del_robot_description')

    robot_gazebo = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'Simulation', 'Gazebo', 'robot_gazebo.launch.xml')
        )
    )

    amcl_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'Simulation', 'AMCL', 'amcl_localization.launch.py')
        )
    )

    hand_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'Real_Robot', 'Hand_Controller', 'hand_controller.launch.py')
        )
    )

    costmap_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'Simulation', 'Costmap', 'costmap_sim.launch.py')
        )
    )

    astar_planner = Node(
        package='food_del_astar_planner',
        executable='astar_planner_node',
        name='astar_planner_node',
        output='screen',
    )

    dwb_controller = Node(
        package='dwb_controller',
        executable='main',
        name='dwb_controller',
        output='screen',
    )

    return LaunchDescription([
        robot_gazebo,
        amcl_localization,
        hand_controller,
        costmap_sim,
        astar_planner,
        dwb_controller,
    ])