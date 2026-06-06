from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bot_navigation_dir = get_package_share_directory("bot_navigation")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    lifecycle_nodes = [
        "controller_server", 
        "planner_server", 
        "smoother_server", 
        "bt_navigator",
        "behavior_server"
        ]

    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        remappings=[
            ("/cmd_vel", "/input_nav/cmd_vel"),
        ],
        parameters=[
            os.path.join(
                get_package_share_directory("bot_navigation"),
                "config",
                "controller_server.yaml"
            ),
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("bot_navigation"),
                "config",
                "planner_server.yaml"
            ),
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("bot_navigation"),
                "config",
                "smoother_server.yaml"
            ),
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_bt_navigator_server = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            os.path.join(
                bot_navigation_dir,
                "config",
                "bt_navigator.yaml"
            ),
            {
                "use_sim_time": use_sim_time,
                "default_nav_to_pose_bt_xml": os.path.join(
                    bot_navigation_dir,
                    "behavior_trees",
                    "simple_replanning_recovery_nav.xml"
                ),
                "default_nav_through_poses_bt_xml": os.path.join(
                    bot_navigation_dir,
                    "behavior_trees",
                    "simple_nav.xml"
                ),
            }
        ]
    )

    nav2_behaviors_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        remappings=[
            ("/cmd_vel", "/input_nav/cmd_vel"),
        ],
        parameters=[
            os.path.join(
                get_package_share_directory("bot_navigation"),
                "config",
                "behavior_server.yaml"
            ),
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        nav2_controller_server,
        nav2_planner_server,
        nav2_smoother_server,
        nav2_bt_navigator_server,
        nav2_behaviors_server,
        nav2_lifecycle_manager
    ])
