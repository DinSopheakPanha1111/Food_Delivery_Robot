import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Safe drop height for base_footprint above the Gazebo ground plane.
# The xacro base_footprint_joint already offsets base_link by spawn_z,
# so this value just gives clearance for a clean spawn — do NOT set it to spawn_z again.
SPAWN_DROP_Z = 0.30


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('food_del_robot_full_description')
    xacro_file = os.path.join(pkg, 'urdf', 'robot_full_design.urdf.xacro')

    use_lidar = LaunchConfiguration('use_lidar').perform(context)

    robot_description = xacro.process_file(
        xacro_file,
        mappings={'use_sim': 'true', 'use_lidar': use_lidar},
    ).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        )
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
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'food_del_robot_full_description',
                    '-z', str(SPAWN_DROP_Z),
                    '-timeout', '60',
                ],
                output='screen',
            )
        ]
    )

    return [gazebo, robot_state_publisher, spawn_entity]


def generate_launch_description():
    pkg_share_parent = os.path.dirname(
        get_package_share_directory('food_del_robot_full_description')
    )
    os.environ['GAZEBO_MODEL_PATH'] = (
        pkg_share_parent + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_lidar',
            default_value='false',
            description='Set to false to disable the LiDAR Gazebo plugin',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
