import os 
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():

    bot_description_dir = get_package_share_directory("bot_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(bot_description_dir, "urdf", "bot.urdf.xacro"),
        description="Absolute path to robot's URDF file"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(bot_description_dir).parent.resolve())]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"
                ]),
                launch_arguments=[
                        (
                            "gz_args", 
                            [
                                " -v 4",
                                " empty.sdf", 
                                " -r"                                
                            ]
                        )
                ]
             )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "bot"]
    )

    # Bridge Gazebo sim clock to ROS 2 /clock
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


    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])
