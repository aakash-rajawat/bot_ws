import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    cyclonedds = SetEnvironmentVariable(
        name="RMW_IMPLEMENTATION",
        value="rmw_cyclonedds_cpp",
    )

    bot_description_dir = get_package_share_directory("bot_description")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                bot_description_dir,
                "rviz",
                "display.rviz",
            ),
        ],
        parameters=[{
            "use_sim_time": True,
        }],
    )

    return LaunchDescription([
        cyclonedds,
        rviz,
    ])
