import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    cyclonedds = SetEnvironmentVariable(
        name="RMW_IMPLEMENTATION",
        value="rmw_cyclonedds_cpp",
    )

    bot_description_dir = get_package_share_directory("bot_description")
    bot_controller_dir = get_package_share_directory("bot_controller")
    bot_worlds_dir = get_package_share_directory("bot_worlds")
    bot_vision_py_dir = get_package_share_directory("bot_vision_py")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(bot_description_dir, "urdf", "bot.urdf.xacro"),
        description="Absolute path to robot URDF/Xacro file",
    )

    world_config_arg = DeclareLaunchArgument(
        "world_config",
        default_value=os.path.join(bot_worlds_dir, "config", "maze.yaml"),
        description="Absolute path to world configuration YAML file for the maze world",
    )

    controller_config_arg = DeclareLaunchArgument(
        "controller_config",
        default_value=os.path.join(
            bot_controller_dir,
            "config",
            "diff_drive_controller_config.yaml",
        ),
        description="Absolute path to diff drive controller config file",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_description_dir, "launch", "headless_gazebo.launch.py")
        ),
        launch_arguments={
            "model": LaunchConfiguration("model"),
            "world_config": LaunchConfiguration("world_config"),
        }.items(),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30.0",
            "--switch-timeout",
            "30.0",
            "--param-file",
            LaunchConfiguration("controller_config"),
        ],
        output="screen",
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "bot_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30.0",
            "--switch-timeout",
            "30.0",
            "--param-file",
            LaunchConfiguration("controller_config"),
        ],
        output="screen",
    )

    joystick_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_controller_dir, "launch", "joystick_teleop.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    safety_stop = Node(
        package="bot_utils",
        executable="safety_stop",
        name="safety_stop",
        output="screen",
        parameters=[{
            "use_sim_time": True,
        }],
    )

    xfeat_lightglue_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bot_vision_py_dir,
                "launch",
                "xfeat_lightglue_server.launch.py",
            )
        )
    )

    vision_frontend = Node(
        package="bot_vision",
        executable="vision_frontend",
        name="vision_frontend",
        output="screen",
        parameters=[{
            "use_sim_time": True,
        }],
    )

    return LaunchDescription([
        cyclonedds,
        model_arg,
        world_config_arg,
        controller_config_arg,
        gazebo,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        joystick_teleop,
        safety_stop,
        xfeat_lightglue_server,
        vision_frontend,
    ])
