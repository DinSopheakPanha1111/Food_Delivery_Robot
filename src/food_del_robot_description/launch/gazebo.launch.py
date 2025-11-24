#!/usr/bin/env python3
#
# Launch Gazebo with the Food Delivery Robot (ROS 2 Control + Diff Drive)
#

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # === Package paths ===
    pkg_food_desc = get_package_share_directory('food_del_robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # === Configurations ===
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.01')

    # === World path ===
    world_path = os.path.join(pkg_food_desc, 'worlds', 'restaurant', 'restaurant.world')
    if not os.path.exists(world_path):
        world_path = os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world')

    # === Xacro (URDF) path ===
    xacro_file = os.path.join(pkg_food_desc, 'urdf', 'my_main.urdf.xacro')
    robot_description_content = os.popen(f'xacro {xacro_file}').read()

    # === ros2_control config path ===
    control_config = os.path.join(pkg_food_desc, 'config', 'ros2_control.yaml')

    # === Gazebo server ===
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # === Gazebo client ===
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # === Robot State Publisher ===
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_content
        }]
    )

    # === Spawn robot in Gazebo ===
    spawn_entity_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'food_del_robot',
            '-topic', 'robot_description',
            '-x', x_pose, '-y', y_pose, '-z', z_pose
        ],
        output='screen'
    )

    # === ROS 2 Control Node ===
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'use_sim_time': True,
                     'robot_description': robot_description_content},
                    control_config],
        output='screen'
    )

    # === Spawners ===
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # === Launch everything ===
    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diff_drive_controller_spawner)

    return ld
