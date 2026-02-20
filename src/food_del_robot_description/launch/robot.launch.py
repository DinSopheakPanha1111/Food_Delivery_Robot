from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    home = os.getenv('HOME')

    # ---------------- Paths ----------------
    urdf_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot_description/urdf/my_main.urdf.xacro')
    rviz_config_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot_description/rviz/food_del_robot.rviz')
    map_yaml_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot/maps/my_map.yaml')
    ekf_config_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot/config/ekf_config.yaml')
    amcl_config_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot/config/amcl_config.yaml')
    planner_config_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot/config/A_star_config.yaml')
    controller_config_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot/config/dwb_config.yaml')
    bt_navigator_config_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot/config/bt_navigator_config.yaml')
    lifecycle_manager_config_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot/config/lifecycle_manager_config.yaml')
    joy_params = os.path.join(get_package_share_directory('food_del_robot_description'), 'config', 'joystick.yaml')

    return LaunchDescription([

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
        # MAP SERVER
        # =========================================================
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': map_yaml_path}]),

        # =========================================================
        # Wait for the map_server to be ready (Introduce a small delay)
        # =========================================================
        TimerAction(
            period=2.0,  # Wait for 2 seconds
            actions=[
                # =========================================================
                # AMCL
                # =========================================================
                Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[amcl_config_path]),

                # =========================================================
                # PLANNER (A*)
                # =========================================================
                Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
                     parameters=[planner_config_path]),

                # =========================================================
                # CONTROLLER (DWB)
                # =========================================================
                Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
                     parameters=[controller_config_path]),

                # =========================================================
                # BT NAVIGATOR
                # =========================================================
                Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
                     parameters=[bt_navigator_config_path]),

                # =========================================================
                # LIFECYCLE MANAGER
                # =========================================================
                Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager', output='screen',
                     parameters=[lifecycle_manager_config_path]),
            ]
        ),
    ])
