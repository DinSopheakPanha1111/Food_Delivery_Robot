from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    home = os.getenv('HOME')

    # ---------------- Paths ----------------
    urdf_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot_description/urdf/my_main.urdf.xacro')
    rviz_config_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot_description/rviz/robot_nav.rviz')
    ekf_config_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot/config/ekf_config.yaml')

    # RPLIDAR launch file path
    rplidar_launch_path = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_a1_launch.py'
    )

    # Joystick launch file path
    joystick_launch_path = os.path.join(
        get_package_share_directory('food_del_robot_description'),
        'launch',
        'joystick.launch.py'
    )

    # Nav2 bringup launch file path
    nav2_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    return LaunchDescription([

        # =========================================================
        # RPLIDAR A1
        # =========================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_path)
        ),

        # =========================================================
        # JOYSTICK TELEOP
        # =========================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joystick_launch_path)
        ),

        # =========================================================
        # EKF
        # =========================================================
        ExecuteProcess(
            cmd=['ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args', '--params-file', ekf_config_path],
            output='screen'
        ),

        # =========================================================
        # Sensors
        # =========================================================
        Node(package='yahboom_imu_10_axis', executable='imu_driver', name='imu_driver', output='screen'),
        Node(package='zlac8015d_driver', executable='drive_and_odom', name='drive_and_odom', output='screen'),

        # =========================================================
        # Robot model
        # =========================================================
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': Command(['xacro ', urdf_path])}]),

        # =========================================================
        # RViz
        # =========================================================
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config_path], output='screen'),

        # =========================================================
        # NAV2 BRINGUP (Navigation Stack)
        # =========================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot_description/maps/outdoor_map.yaml'),
                'use_sim_time': 'False'
            }.items()
        ),

    ])