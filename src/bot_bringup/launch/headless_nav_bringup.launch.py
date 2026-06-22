import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bot_bringup_dir = get_package_share_directory("bot_bringup")
    bot_controller_dir = get_package_share_directory("bot_controller")
    bot_description_dir = get_package_share_directory("bot_description")
    bot_mapping_dir = get_package_share_directory("bot_mapping")
    bot_navigation_dir = get_package_share_directory("bot_navigation")
    bot_worlds_dir = get_package_share_directory("bot_worlds")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(bot_description_dir, "urdf", "bot.urdf.xacro"),
        description="Absolute path to robot URDF/Xacro file",
    )

    world_config_arg = DeclareLaunchArgument(
        "world_config",
        default_value=os.path.join(bot_worlds_dir, "config", "maze.yaml"),
        description="Absolute path to world configuration YAML file",
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

    wheel_odometry_parametric_config_arg = DeclareLaunchArgument(
        "wheel_odometry_parametric_config",
        default_value=os.path.join(
            bot_controller_dir,
            "config",
            "wheel_odometry_parametric.yaml",
        ),
        description="Absolute path to wheel odometry parametric config file",
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            bot_mapping_dir,
            "config",
            "slam_toolbox.yaml",
        ),
        description="Absolute path to slam toolbox config file",
    )

    no_gui_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_bringup_dir, "launch", "no_gui_bringup.launch.py")
        ),
        launch_arguments={
            "model": LaunchConfiguration("model"),
            "world_config": LaunchConfiguration("world_config"),
            "controller_config": LaunchConfiguration("controller_config"),
            "wheel_odometry_parametric_config": LaunchConfiguration(
                "wheel_odometry_parametric_config"
            ),
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_mapping_dir, "launch", "slam.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam_config": LaunchConfiguration("slam_config"),
        }.items(),
    )

    navigation = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bot_navigation_dir, "launch", "navigation.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        model_arg,
        world_config_arg,
        controller_config_arg,
        wheel_odometry_parametric_config_arg,
        slam_config_arg,
        no_gui_bringup,
        slam,
        navigation,
    ])
