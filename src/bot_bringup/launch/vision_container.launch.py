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

    ua_wheel_odom_config_arg = DeclareLaunchArgument(
        "ua_wheel_odom_config",
        default_value=os.path.join(
            bot_controller_dir,
            "config",
            "ua_wheel_odom.yaml",
        ),
        description="Absolute path to uncertainty-aware wheel odometry config file",
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

    ua_lidar_point_cloud = Node(
        package="bot_multisensor_odometry",
        executable="ua_lidar_point_cloud",
        name="ua_lidar_point_cloud",
        output="screen",
        parameters=[{
            "use_sim_time": True,
        }],
    )

    ua_wheel_odom = Node(
        package="bot_controller",
        executable="ua_wheel_odom",
        name="ua_wheel_odom",
        output="screen",
        parameters=[
            LaunchConfiguration("ua_wheel_odom_config"),
            {
                "use_sim_time": True,
            },
        ],
    )

    mle_relative_pose_server_lidar = Node(
        package="bot_multisensor_odometry",
        executable="mle_relative_pose_server_node",
        name="mle_relative_pose_server_lidar",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "relative_pose_action_name": "/mle_relative_pose_lidar",
        }],
    )

    mle_relative_pose_server_triangulation = Node(
        package="bot_multisensor_odometry",
        executable="mle_relative_pose_server_node",
        name="mle_relative_pose_server_triangulation",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "relative_pose_action_name": "/mle_relative_pose_triangulation",
        }],
    )

    mle_relative_pose_client_lidar = Node(
        package="bot_multisensor_odometry",
        executable="mle_relative_pose_client_node",
        name="mle_relative_pose_client_lidar",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "topic_point_cloud": "/ua_lidar/ua_point_cloud",
            "topic_relative_pose": "/bot_controller/relative_pose_mle_lidar",
            "relative_pose_action_name": "/mle_relative_pose_lidar",
        }],
    )

    mle_relative_pose_client_triangulation = Node(
        package="bot_multisensor_odometry",
        executable="mle_relative_pose_client_node",
        name="mle_relative_pose_client_triangulation",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "topic_point_cloud": "/ua_triangulation/pointswithcovariance",
            "topic_relative_pose": "/bot_controller/relative_pose_mle_triangulation",
            "relative_pose_action_name": "/mle_relative_pose_triangulation",
        }],
    )

    relative_pose_odometry_lidar = Node(
        package="bot_multisensor_odometry",
        executable="relative_pose_odometry",
        name="relative_pose_odometry_lidar",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "mle_pose_topic": "/bot_controller/relative_pose_mle_lidar",
            "odom_mle_topic": "/bot_controller/odom_mle_lidar",
        }],
    )

    relative_pose_odometry_triangulation = Node(
        package="bot_multisensor_odometry",
        executable="relative_pose_odometry",
        name="relative_pose_odometry_triangulation",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "mle_pose_topic": "/bot_controller/relative_pose_mle_triangulation",
            "odom_mle_topic": "/bot_controller/odom_mle_triangulation",
        }],
    )

    gtsam_pose_slam = Node(
        package="bot_localization",
        executable="gtsam_pose_slam",
        name="gtsam_pose_slam",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "initial_pose_x": 1.0,
            "initial_pose_y": 1.0,
            "initial_pose_yaw": 0.0,
        }],
    )

    odom_error = Node(
        package="bot_evaluation",
        executable="odom_error",
        name="odom_error",
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
        ua_wheel_odom_config_arg,
        gazebo,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        joystick_teleop,
        safety_stop,
        xfeat_lightglue_server,
        vision_frontend,
        ua_lidar_point_cloud,
        ua_wheel_odom,
        mle_relative_pose_server_lidar,
        mle_relative_pose_server_triangulation,
        mle_relative_pose_client_lidar,
        mle_relative_pose_client_triangulation,
        relative_pose_odometry_lidar,
        relative_pose_odometry_triangulation,
        gtsam_pose_slam,
        odom_error
    ])
