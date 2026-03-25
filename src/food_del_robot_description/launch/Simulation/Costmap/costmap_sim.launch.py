from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.descriptions import ParameterFile


COSTMAP_CONFIG = "/home/panha/Food_Delivery_Robot/src/food_del_robot/config/Simulation/Costmap/local_costmap_config.yaml"


def generate_launch_description():

    costmap_params_arg = DeclareLaunchArgument(
        "costmap_params",
        default_value=COSTMAP_CONFIG,
        description="Full path to the local costmap YAML config file",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    costmap_params = LaunchConfiguration("costmap_params")
    use_sim_time = LaunchConfiguration("use_sim_time")

    costmap_node = LifecycleNode(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        name="costmap",
        namespace="costmap",
        output="screen",
        parameters=[
            ParameterFile(costmap_params, allow_substs=True),
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        costmap_params_arg,
        use_sim_time_arg,
        costmap_node,
    ])