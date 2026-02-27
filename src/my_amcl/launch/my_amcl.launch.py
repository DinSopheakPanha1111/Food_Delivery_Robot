import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    home = os.getenv('HOME')

    params_file = os.path.join(
        get_package_share_directory('my_amcl'),
        'config',
        'my_amcl_params.yaml'
    )

    map_file = os.path.join(
        home,
        'Food_Delivery_Robot/src/food_del_robot/maps/outdoor_map.yaml'
    )

    return LaunchDescription([

        # ── Map Server ───────────────────────────────────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file, 'use_sim_time': False}]
        ),

        # ── Lifecycle manager just for map_server ────────────────
        # map_server is a lifecycle node — it needs to be activated
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),

        # ── Our custom AMCL ──────────────────────────────────────
        Node(
            package='my_amcl',
            executable='my_amcl_node',
            name='my_amcl',
            output='screen',
            parameters=[params_file]
        ),
    ])
