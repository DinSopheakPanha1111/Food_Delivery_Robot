from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    home = os.getenv('HOME')

    # ── Paths ─────────────────────────────────────────────────────────────────
    urdf_path = os.path.join(
        home, 'Food_Delivery_Robot/src/food_del_robot_description/urdf/Real_Robot/my_main.urdf.xacro')
    rviz_config_path = os.path.join(
        home, 'Food_Delivery_Robot/src/food_del_robot_description/rviz/food_del_robot.rviz')
    map_yaml_path = os.path.join(
        home, 'Food_Delivery_Robot/src/food_del_robot/maps/Class_map/class_map.yaml')
    amcl_config_path = os.path.join(
        home, 'Food_Delivery_Robot/src/food_del_robot/config/Real_Robot/AMCL/amcl_config.yaml')
    smac_config_path = os.path.join(
        home, 'Food_Delivery_Robot/src/food_del_robot/config/Real_Robot/Planner_Server/smac_config.yaml')
    local_costmap_config_path = os.path.join(
        home, 'Food_Delivery_Robot/src/food_del_robot/config/Real_Robot/Controller_Server/local_costmap_real_config.yaml')
    lifecycle_manager_config_path = os.path.join(
        home, 'Food_Delivery_Robot/src/food_del_robot/config/Real_Robot/Lifecycle_manager/lifecycle_manager_smac.yaml')

    rplidar_launch_path = os.path.join(
        get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
    handcontroller_launch_path = os.path.join(
        get_package_share_directory('food_del_robot_description'),
        'launch', 'Real_Robot', 'Hand_Controller', 'hand_controller.launch.py')

    return LaunchDescription([

        # =====================================================================
        # RPLIDAR A1
        # =====================================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_path)
        ),

        # =====================================================================
        # JOYSTICK
        # =====================================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(handcontroller_launch_path)
        ),

        # =====================================================================
        # Sensors
        # =====================================================================
        Node(package='yahboom_imu_10_axis', executable='imu_driver',
             name='imu_driver', output='screen'),
        Node(package='zlac8015d_driver', executable='drive_and_odom_v2',
             name='drive_and_odom_v2', output='screen'),

        # =====================================================================
        # Robot model
        # =====================================================================
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': Command(['xacro ', urdf_path])}]),

        # =====================================================================
        # RViz
        # =====================================================================
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', rviz_config_path], output='screen'),

        # =====================================================================
        # MAP SERVER  (lifecycle — managed by lifecycle_manager_smac)
        # =====================================================================
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='screen',
             parameters=[{'yaml_filename': map_yaml_path}]),

        # =====================================================================
        # Wait 2 s for map_server + TF to be ready, then bring up navigation
        # =====================================================================
        TimerAction(
            period=2.0,
            actions=[

                # =============================================================
                # AMCL  (lifecycle — managed by lifecycle_manager_smac)
                # =============================================================
                Node(package='nav2_amcl', executable='amcl',
                     name='amcl', output='screen',
                     parameters=[amcl_config_path]),

                # =============================================================
                # PLANNER SERVER — SmacPlanner2D
                # (lifecycle — managed by lifecycle_manager_smac)
                # =============================================================
                Node(package='nav2_planner', executable='planner_server',
                     name='planner_server', output='screen',
                     parameters=[smac_config_path]),

                # =============================================================
                # LIFECYCLE MANAGER
                # Activates: map_server → amcl → planner_server
                # =============================================================
                Node(package='nav2_lifecycle_manager',
                     executable='lifecycle_manager',
                     name='lifecycle_manager', output='screen',
                     parameters=[lifecycle_manager_config_path]),

                # =============================================================
                # GOAL BRIDGE
                # Converts /goal_pose → ComputePathToPose action → /plan
                # =============================================================
                Node(package='food_del_goal_bridge', executable='goal_bridge',
                     name='goal_bridge', output='screen'),

                # =============================================================
                # CUSTOM DWB CONTROLLER
                # Subscribes /plan + /odom, publishes /cmd_vel
                # =============================================================
                Node(package='dwb_controller', executable='main',
                     name='dwb_controller', output='screen',
                     parameters=[local_costmap_config_path]),
            ]
        ),
    ])
