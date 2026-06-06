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
    bot_worlds_dir = get_package_share_directory("bot_worlds")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(bot_description_dir, "urdf", "bot.urdf.xacro"),
        description="Absolute path to robot URDF/Xacro file",
    )

    world_config_arg = DeclareLaunchArgument(
        "world_config",
        default_value=os.path.join(
            bot_worlds_dir,
            "config",
            "maze.yaml",
        ),
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
            # "wheel_odometry_parametric_bad.yaml",
        ),
        description="Absolute path to wheel odometry parametric config file",
    )

    odometry_ekf_config_arg = DeclareLaunchArgument(
        "odometry_ekf_config",
        default_value=os.path.join(
            bot_localization_dir,
            "config",
            "odometry_ekf.yaml",
        ),
        description="Absolute path to odometry EKF config file",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_description_dir, "launch", "gazebo_new.launch.py")
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
            "--param-file",
            LaunchConfiguration("controller_config"),
        ],
        output="screen",
    )

    wheel_odometry_parametric = Node(
        package="bot_controller",
        executable="wheel_odometry_parametric",
        name="wheel_odometry_parametric",
        output="screen",
        parameters=[LaunchConfiguration("wheel_odometry_parametric_config")],
    )

    odometry_ekf = Node(
        package="bot_localization",
        executable="odometry_ekf",
        name="odometry_ekf",
        output="screen",
        parameters=[LaunchConfiguration("odometry_ekf_config")],
    )

    joystick_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_controller_dir, "launch", "joystick_teleop.launch.py")
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
        world_config_arg,
        controller_config_arg,
        wheel_odometry_parametric_config_arg,
        odometry_ekf_config_arg,
        gazebo,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        wheel_odometry_parametric,
        odometry_ekf,
        joystick_teleop,
        rviz,
    ])
