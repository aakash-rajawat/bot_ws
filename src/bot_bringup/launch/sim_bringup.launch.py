import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    bot_description_dir = get_package_share_directory("bot_description")
    bot_controller_dir = get_package_share_directory("bot_controller")
    bot_localization_dir = get_package_share_directory("bot_localization")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(bot_description_dir, "urdf", "bot.urdf.xacro"),
        description="Absolute path to robot URDF/Xacro file",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_description_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "model": LaunchConfiguration("model"),
        }.items(),
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_controller_dir, "launch", "controller.launch.py")
        ),
        launch_arguments={
        }.items(),
    )

    joystick_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_controller_dir, "launch", "joystick_teleop.launch.py")
        )
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_localization_dir, "launch", "local_localization.launch.py")
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(bot_description_dir, "rviz", "display.rviz"),
        ],
    )

    return LaunchDescription([
        model_arg,
        gazebo,
        controller,
        joystick_teleop,
        localization,
        rviz,
    ])
