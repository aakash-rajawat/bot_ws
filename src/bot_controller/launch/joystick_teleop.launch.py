from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    bot_controller_dir = get_package_share_directory("bot_controller")

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulated time"
    )

    joy_node =  Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joystick",
        parameters=[
            os.path.join(get_package_share_directory("bot_controller"), "config", "joy_config.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[
            os.path.join(get_package_share_directory("bot_controller"), "config", "joy_teleop.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("twist_mux"),
                "launch",
                "twist_mux_launch.py"
            )
        ),
        launch_arguments={
            "cmd_vel_out": "/bot_controller/cmd_vel_unstamped",
            "config_topics": os.path.join(bot_controller_dir, "config", "twist_mux_topics.yaml"),
            "config_locks": os.path.join(bot_controller_dir, "config", "twist_mux_locks.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items()
    )

    twist_relay_node = Node(
        package="bot_controller",
        executable="twist_relay",
        name="twist_relay",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        joy_node,
        joy_teleop,
        twist_mux_launch,
        twist_relay_node
    ])
