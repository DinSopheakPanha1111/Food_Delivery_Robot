#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    desc = get_package_share_directory('food_del_robot_description')

    xacro_file = os.path.join(desc, 'urdf', 'food_del_robot.urdf.xacro')
    world_path = os.path.join(desc, 'worlds', 'Restaurant', 'Restaurant.sdf')

    # Set GAZEBO_MODEL_PATH so Gazebo can resolve package:// mesh URIs
    pkg_share_parent = os.path.dirname(desc)
    os.environ['GAZEBO_MODEL_PATH'] = (
        pkg_share_parent + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    robot_description = xacro.process_file(xacro_file, mappings={'use_sim': 'true'}).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    spawn_entity = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'my_robot', '-z', '0.30', '-timeout', '60'],
                output='screen',
            )
        ],
    )

    hmi = ExecuteProcess(
        cmd=[
            'python3',
            '/home/panha/Food_Delivery_Robot/src/food_del_robot_hmi/food_del_robot_hmi/food_del_robot_main_hmi.py'
        ],
        output='screen',
        name='food_del_robot_hmi',
    )

    return LaunchDescription([gazebo, robot_state_publisher, spawn_entity, hmi])