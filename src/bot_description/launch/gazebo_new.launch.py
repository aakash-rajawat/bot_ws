import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def configure_gazebo(context, *args, **kwargs):
    bot_description_dir = get_package_share_directory("bot_description")
    bot_worlds_dir = get_package_share_directory("bot_worlds")

    model_path = LaunchConfiguration("model").perform(context)
    world_config_path = LaunchConfiguration("world_config").perform(context)

    with open(world_config_path, "r", encoding="utf-8") as file:
        world_config = yaml.safe_load(file)

    world_sdf = os.path.join(
        bot_worlds_dir,
        world_config["world"]["sdf"]
    )

    spawn_x = str(world_config["spawn"]["x"])
    spawn_y = str(world_config["spawn"]["y"])
    spawn_z = str(world_config["spawn"]["z"])
    spawn_yaw = str(world_config["spawn"]["yaw"])

    robot_description = ParameterValue(
        Command(["xacro ", model_path]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description,
            "use_sim_time": True}
        ]
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(bot_description_dir).parent.resolve()),
            os.pathsep,
            str(Path(bot_worlds_dir).parent.resolve()),
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
            "/gz_sim.launch.py"
        ]),
        launch_arguments=[
            (
                "gz_args",
                [
                    " -v 4",
                    f" {world_sdf}",
                    " -r"
                ]
            )
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "bot",
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-Y", spawn_yaw,
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/model/bot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/bot/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance",
            "/right_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/right_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/left_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/left_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/imu", "/imu/out"),
        ],
        output="screen",
    )

    return [
        robot_state_publisher,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ]


def generate_launch_description():
    bot_description_dir = get_package_share_directory("bot_description")
    bot_worlds_dir = get_package_share_directory("bot_worlds")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(bot_description_dir, "urdf", "bot.urdf.xacro"),
        description="Absolute path to robot's URDF file"
    )

    world_config_arg = DeclareLaunchArgument(
        name="world_config",
        default_value=os.path.join(bot_worlds_dir, "config", "maze.yaml"),
        description="Absolute path to world configuration YAML file"
    )


    return LaunchDescription([
        model_arg,
        world_config_arg,
        OpaqueFunction(function=configure_gazebo)
    ])
