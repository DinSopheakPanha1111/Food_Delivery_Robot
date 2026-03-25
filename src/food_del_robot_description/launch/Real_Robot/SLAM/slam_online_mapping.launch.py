import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():

    home = os.getenv('HOME')

    # --- Paths ---
    urdf_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot_description/urdf/Real_Robot/my_main.urdf.xacro')
    rviz_config_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot_description/rviz/food_del_robot.rviz')
    slam_params_path   = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot/config/Real_Robot/SLAM/slam_online_mapping.yaml')
    rplidar_launch_path = os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
    handcontroller_launch_path = os.path.join(get_package_share_directory('food_del_robot'), 'launch', 'Real_Robot','Hand_Controller','hand_controller.launch.py')

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
            executable='drive_and_odom_v2',
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
        # SLAM Toolbox (online async)
        # =========================================================
        ExecuteProcess(
            cmd=['ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node',
                 '--ros-args', '--params-file', slam_params_path],
            output='screen'
        ),

        # =========================================================
        # Robot model
        # =========================================================
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': Command(['xacro ', urdf_path])}]),
        
        # =========================================================
        # RViz
        # =========================================================
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config_path], output='screen'),

    ])
