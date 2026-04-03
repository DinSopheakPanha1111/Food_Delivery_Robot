#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg       = get_package_share_directory('food_del_robot_description')
    pkg_robot = get_package_share_directory('food_del_robot')

    smac_sim_config = os.path.join(
        pkg_robot, 'config', 'Simulation', 'Planner', 'smac_sim_config.yaml'
    )
    local_costmap_config = os.path.join(
        pkg_robot, 'config', 'Simulation', 'Costmap', 'local_costmap_config.yaml'
    )
    lifecycle_planner_config = os.path.join(
        pkg_robot, 'config', 'Simulation', 'Lifecycle_manager', 'lifecycle_manager_planner.yaml'
    )

    # ── Included launch files (unchanged from robot_sim_with_amcl.launch.py) ──
    robot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'Simulation', 'Gazebo', 'robot_gazebo.launch.py')
        )
    )

    # Handles map_server + amcl + their lifecycle_manager (node_names: map_server, amcl)
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

    # ── Planner server — SmacPlanner2D ────────────────────────────────────────
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[smac_sim_config, {'use_sim_time': True}],
    )

    # Dedicated lifecycle manager for planner_server only.
    # amcl_localization already manages map_server + amcl.
    lifecycle_manager_planner = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planner',
        output='screen',
        parameters=[lifecycle_planner_config],
    )

    # ── Goal bridge ───────────────────────────────────────────────────────────
    # /goal_pose → ComputePathToPose action → /plan
    goal_bridge = Node(
        package='food_del_goal_bridge',
        executable='goal_bridge',
        name='goal_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── Path recorder ─────────────────────────────────────────────────────────
    # Records /odom poses (transformed to map frame) → /actual_path
    # Add a Path display in RViz on /actual_path to visualize the actual trajectory.
    path_recorder = Node(
        package='food_del_goal_bridge',
        executable='path_recorder',
        name='path_recorder',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── Custom DWB controller (unchanged) ─────────────────────────────────────
    dwb_controller = Node(
        package='dwb_controller',
        executable='main',
        name='dwb_controller',
        parameters=[local_costmap_config, {'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        robot_gazebo,
        amcl_localization,
        hand_controller,
        dwb_controller,
        path_recorder,
        # Wait 6 s for map_server (starts at 3 s) and TF to be ready
        # before bringing up planner_server and its lifecycle manager.
        TimerAction(
            period=6.0,
            actions=[
                planner_server,
                lifecycle_manager_planner,
                goal_bridge,
            ]
        ),
    ])
