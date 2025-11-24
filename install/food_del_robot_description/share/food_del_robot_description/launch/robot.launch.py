from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
import os

def generate_launch_description():
    robot_description_path = get_package_share_path('food_del_robot_description')

    urdf_path = os.path.join(robot_description_path, 'urdf', 'my_main.urdf.xacro')
    rviz_config_path = os.path.join(robot_description_path, 'rviz', 'food_del_robot.rviz')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_controllers = os.path.join(robot_description_path, 'config', 'food_del_robot.yaml')
    map_path = os.path.join(robot_description_path, 'maps', 'random.yaml')

    # --- Robot Description ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }],
    )

    # --- ROS2 Control ---
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            robot_controllers
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    # --- RViz2 ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
    )

    # --- Map Server (delayed) ---
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_path
        }]
    )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # Delay launching map_server + lifecycle_mgr for 3 seconds
    delayed_map_nodes = TimerAction(
        period=3.0,
        actions=[map_server, lifecycle_mgr],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        rviz_node,                # Start RViz first
        delayed_map_nodes,        # Then start map_server after delay
    ])
