#!/usr/bin/env python3
import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('food_del_robot_bringup')
    desc = get_package_share_directory('food_del_robot_description')
    rtabmap_launch_pkg = get_package_share_directory('rtabmap_launch')

    # --------------------------------------------------
    # Paths
    # --------------------------------------------------
    xacro_file = os.path.join(desc, 'urdf', 'food_del_robot.urdf.xacro')
    world_path = os.path.join(desc, 'worlds', 'Restaurant', 'Restaurant')
    rtabmap_db_path = '/tmp/rtabmap.db'

    # Gazebo mesh path
    pkg_share_parent = os.path.dirname(desc)
    os.environ['GAZEBO_MODEL_PATH'] = (
        pkg_share_parent + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    robot_description = xacro.process_file(
        xacro_file,
        mappings={'use_sim': 'true'}
    ).toxml()

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Launch RTAB-Map in localization mode instead of mapping mode'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz from RTAB-Map only'
    )

    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/camera/image_raw',
        description='RGB image topic'
    )

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/depth/image_raw',
        description='Depth image topic'
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera_info',
        description='Camera info topic'
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Laser scan topic'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    rviz = LaunchConfiguration('rviz')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')

    # --------------------------------------------------
    # FORCE DELETE old RTAB-Map database before launch
    # --------------------------------------------------
    reset_rtabmap_db = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'rm -f {rtabmap_db_path} {rtabmap_db_path}-shm {rtabmap_db_path}-wal {rtabmap_db_path}-journal'
        ],
        output='screen'
    )

    # --------------------------------------------------
    # Gazebo with YOUR world
    # --------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_path
        }.items(),
    )

    # --------------------------------------------------
    # Robot state publisher
    # --------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
    )

    # --------------------------------------------------
    # Spawn robot
    # --------------------------------------------------
    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'food_deli_robot',
                    '-z', '0.30',
                    '-timeout', '60'
                ],
                output='screen',
            )
        ],
    )

    # --------------------------------------------------
    # Hand controller
    # --------------------------------------------------
    hand_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup,
                'launch',
                'hand_controller.launch.py'
            )
        )
    )

    # --------------------------------------------------
    # RTAB-Map
    # --------------------------------------------------
    rtabmap = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        rtabmap_launch_pkg,
                        'launch',
                        'rtabmap.launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,

                    'rgb_topic': rgb_topic,
                    'depth_topic': depth_topic,
                    'camera_info_topic': camera_info_topic,
                    'scan_topic': scan_topic,
                    'odom_topic': odom_topic,

                    'approx_sync': 'true',
                    'subscribe_scan': 'true',
                    'subscribe_depth': 'true',
                    'subscribe_rgb': 'true',

                    'localization': localization,

                    'rviz': rviz,
                    'rtabmap_viz': 'true',

                    'delete_db_on_start': 'true',
                    'database_path': rtabmap_db_path,

                    'frame_id': 'base_link',
                    'map_frame_id': 'map',
                    'publish_tf': 'true',
                    'grid_map': 'true',
                }.items()
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        localization_arg,
        rviz_arg,
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        scan_topic_arg,
        odom_topic_arg,

        reset_rtabmap_db,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        hand_controller,
        rtabmap,
    ])