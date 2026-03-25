import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('food_del_robot_description')
    pkg2 = get_package_share_directory('food_del_robot')

    # ── Launch Arguments ──────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg2, 'config','Real_Robot','SLAM', 'slam_online_mapping.yaml'),
        description='Path to slam_toolbox parameters file'
    )

    use_sim_time    = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # ── 1. Full Gazebo simulation (your existing launch) ──────────────────────
    robot_gazebo = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'Simulation', 'Gazebo', 'robot_gazebo.launch.xml')
        )
    )

    # ── 2. slam_toolbox – async mapping mode ─────────────────────────────────
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time':   use_sim_time,
            'slam_params_file': slam_params_file,
        }.items()
    )
    return LaunchDescription([
        use_sim_time_arg,
        slam_params_file_arg,
        robot_gazebo,       # Gazebo + robot bringup first
        slam_toolbox_launch,  # then SLAM on top
    ])