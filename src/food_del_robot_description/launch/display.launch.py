from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os

def generate_launch_description():

    home = os.getenv('HOME')

    # ---------------- Paths ----------------
    urdf_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot_description/urdf/my_sim_robot.urdf.xacro')
    rviz_config_path = os.path.join(home, 'Food_Delivery_Robot/src/food_del_robot_description/rviz/robot.rviz')
    
    return LaunchDescription([
        # =========================================================
        # Robot model
        # =========================================================
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': Command(['xacro ', urdf_path])}]),
        
        # =========================================================
        # Robot state publisher GUI
        # =========================================================
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', output='screen'),
        
        # =========================================================
        # RViz
        # =========================================================
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config_path], output='screen'),
    ])