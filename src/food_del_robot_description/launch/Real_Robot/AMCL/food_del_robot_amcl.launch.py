from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():

    home = os.getenv('HOME')

    # ---------------- Paths ----------------
    map_yaml_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot/maps/Outdoor_map/outdoor_map.yaml')
    amcl_config_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot/config/Real_Robot/AMCL/amcl_config.yaml')
    lifecycle_manager_config_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot/config/Lifecycle_manager/lifecycle_manager_for_amcl.yaml')
    rplidar_launch_path = os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
    handcontroller_launch_path = os.path.join(get_package_share_directory('food_del_robot_description'), 'launch', 'Real_Robot','Hand_Controller', 'hand_controller.launch.py')
    urdf_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot_description/urdf/Real_Robot/my_main.urdf.xacro')
    rviz_config_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot_description/rviz/robot_amcl.rviz')
    
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
            PythonLaunchDescriptionSource(handcontroller_launch_path)
        ),

        # =========================================================
        # Motor Driver
        # =========================================================
        Node(
            package='zlac8015d_driver',
            executable='drive_and_odom',
            name='drive_and_odom_v2',
            output='screen'
        ),

        # =========================================================
        # IMU
        # =========================================================
        Node(
            package='yahboom_imu_10_axis',
            executable='imu_driver',
            name='imu_driver',
            output='screen'
        ),

        # =========================================================
        # MAP SERVER
        # =========================================================
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': map_yaml_path}]),

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
        # Wait for the map_server to be ready (Introduce a small delay)
        # =========================================================
        TimerAction(
            period=2.0,
            actions=[
                # =========================================================
                # AMCL
                # =========================================================
                Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
                     parameters=[amcl_config_path]),

                # =========================================================
                # LIFECYCLE MANAGER
                # =========================================================
                Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager', output='screen',
                     parameters=[lifecycle_manager_config_path]),
            ]
        ),
    ])
