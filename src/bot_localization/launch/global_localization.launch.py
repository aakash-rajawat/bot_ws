from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_descritpion():

    map_path = PathJoinSubstitution([
        get_package_share_directory("bot_mapping"),
        "maps",
        map_name,
        "map.yaml"
    ])

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="maze"
    )
    map_name = LaunchConfiguration("map_name")

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(
            get_package_share_directory("bot_localization"),
            "config",
            "amcl.yaml"
        )
    )
    amcl_config = LaunchConfiguration("amcl_config")

    lifecycle_nodes = ["map_server", "amcl"]

    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            # amcl_config,
            {"use_sim_time": use_sim_time}
            ]
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="map_server",
        output="screen",
        parameters=[
            {"node_name": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ]
    )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        amcl_config_arg,
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager
    ])
