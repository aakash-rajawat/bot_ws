import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    bot_description_dir = get_package_share_directory("bot_description")
    bot_controller_dir = get_package_share_directory("bot_controller")
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
        ),
        description="Absolute path to wheel odometry parametric config file",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to launch RViz",
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

    wheel_odometry_parametric = Node(
        package="bot_controller",
        executable="wheel_odometry_parametric",
        name="wheel_odometry_parametric",
        output="screen",
        parameters=[LaunchConfiguration("wheel_odometry_parametric_config")],
    )

    joystick_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_controller_dir, "launch", "joystick_teleop.launch.py")
        )
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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=[
            "-d",
            os.path.join(
                bot_description_dir,
                "rviz",
                "nav2_default_view.rviz",
            ),
        ],
        parameters=[{
            "use_sim_time": True,
        }],
    )

    return LaunchDescription([
        model_arg,
        world_config_arg,
        controller_config_arg,
        wheel_odometry_parametric_config_arg,
        use_rviz_arg,
        gazebo,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        wheel_odometry_parametric,
        joystick_teleop,
        safety_stop,
        rviz,
    ])
