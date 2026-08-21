from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    frame_id = LaunchConfiguration("frame_id")
    imu_topic = LaunchConfiguration("imu_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("imu_bringup"), "config", "wt901c.yaml"]
                ),
            ),
            DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("frame_id", default_value="imu_link"),
            DeclareLaunchArgument("imu_topic", default_value="imu/data"),
            Node(
                package="wt901c_driver",
                executable="imu_driver",
                name="imu_driver",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "port": port,
                        "baudrate": ParameterValue(baudrate, value_type=int),
                        "frame_id": frame_id,
                    },
                ],
                remappings=[("imu/data", imu_topic)],
            ),
        ]
    )
