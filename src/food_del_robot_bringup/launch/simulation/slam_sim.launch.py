#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('food_del_robot_bringup')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock',
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(bringup, 'config', 'real_robot', 'slam', 'slam_online_mapping.yaml'),
        description='Path to slam_toolbox parameters file',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    robot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'simulation', 'gazebo_in_restaurent_also_rviz.launch.py'))
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    hand_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup,
                'launch',
                'hand_controller.launch.py'
            )
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_file_arg,
        robot_gazebo,
        slam_toolbox_launch,
        hand_controller_launch,
    ])