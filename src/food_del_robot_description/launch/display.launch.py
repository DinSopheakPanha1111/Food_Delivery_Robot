#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # URDF and RViz file paths
    urdf_path = os.path.join(
        os.getenv('HOME'),
        'Food_Delivery_Robot/src/food_del_robot_description/urdf/my_main.urdf.xacro'
    )

    rviz_config_path = os.path.join(
        os.getenv('HOME'),
        'Food_Delivery_Robot/src/food_del_robot_description/rviz/food_del_robot.rviz'
    )

    # Joy parameters file
    joy_params = os.path.join(
        get_package_share_directory('food_del_robot_description'), 
        'config', 
        'joystick.yaml'
    )

    # Declare the launch argument for EKF config file
    return LaunchDescription([

        # Declare the ekf_config argument with a default value
        DeclareLaunchArgument(
            'ekf_config',
            default_value='/home/panha/Food_Delivery_Robot/src/food_del_robot/config/ekf_config.yaml',
            description='Path to EKF configuration file'
        ),

        # IMU node
        Node(
            package='yahboom_imu_10_axis',
            executable='imu_driver',
            name='imu_driver',
        ),

        # Odometry node
        Node(
            package='zlac8015d_driver',
            executable='drive_and_odom',
            name='drive_and_odom',
        ),

        # Launch EKF Node externally using ros2 run
        ExecuteProcess(
            cmd=['ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args', '--params-file', LaunchConfiguration('ekf_config')],
            output='screen'
        ),

        # Joy node for joystick
        Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
        ),

        # Teleop node for joystick control
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params],
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),

        # Robot State Publisher (loads URDF via xacro)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path])
            }]
        ),

        # RViz2 with config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
    ])
