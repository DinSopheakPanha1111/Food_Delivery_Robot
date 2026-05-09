#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    desc = get_package_share_directory('food_del_robot_description')
    bringup = get_package_share_directory('food_del_robot_bringup')

    urdf_path = os.path.join(desc, 'urdf', 'food_del_robot.urdf.xacro')
    rviz_config_path = os.path.join(desc, 'rviz', 'food_del_sim_v5.rviz')
    map_yaml_path = os.path.join(bringup, 'maps', 'Class_map', 'class_map.yaml')
    amcl_config_path = os.path.join(bringup, 'config', 'real_robot', 'amcl', 'amcl_config.yaml')
    controller_config_path = os.path.join(bringup, 'config', 'real_robot', 'controller_server', 'vector_pursuit_config.yaml')
    planner_config_path = os.path.join(bringup, 'config', 'real_robot', 'planner_server', 'Smac_planner_config.yaml')
    bt_navigator_config_path = os.path.join(bringup, 'config', 'real_robot', 'bt_navigator', 'bt_navigator_config.yaml')
    lifecycle_config_path = os.path.join(bringup, 'config', 'real_robot', 'lifecycle_manager', 'lifecycle_manager_config.yaml')
    behaviour_config_path = os.path.join(bringup, 'config', 'real_robot', 'behaviour', 'behaviour_config.yaml')
    rplidar_launch_path = os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
    hand_controller_launch = os.path.join(bringup, 'launch', 'hand_controller.launch.py')
    realsense_launch_path = os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')  # ← NEW

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(rplidar_launch_path)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(hand_controller_launch)),
        IncludeLaunchDescription(                                    # ← NEW
            PythonLaunchDescriptionSource(realsense_launch_path),   # ← NEW
            launch_arguments={'pointcloud.enable': 'true'}.items()  # ← NEW
        ),                                                           # ← NEW
        Node(package='yahboom_imu_10_axis', executable='imu_driver', name='imu_driver', output='screen'),
        Node(package='zlac8015d_driver', executable='drive_and_odom_v2', name='drive_and_odom_v2', output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str)}],
        ),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config_path], output='screen'),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': map_yaml_path}]),
        TimerAction(period=2.0, actions=[
            Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
                 parameters=[amcl_config_path]),
            Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
                 parameters=[planner_config_path]),
            Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
                 parameters=[controller_config_path]),
            Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
                 parameters=[behaviour_config_path]),
            Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
                 parameters=[bt_navigator_config_path]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager', output='screen',
                 parameters=[lifecycle_config_path]),
        ]),
    ])