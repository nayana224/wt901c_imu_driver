from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    frame_id = LaunchConfiguration("frame_id")
    parent_frame = LaunchConfiguration("parent_frame")

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("imu_bringup"), "launch", "imu_bringup.launch.py"]
            )
        ),
        launch_arguments={
            "port": port,
            "baudrate": baudrate,
            "frame_id": frame_id,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("frame_id", default_value="imu_link"),
            DeclareLaunchArgument("parent_frame", default_value="world"),
            driver_launch,
            # Bench visualization only. Production robots should provide base_link -> imu_link
            # from URDF or their own static transform instead of using this world frame.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="imu_test_static_tf",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    parent_frame,
                    frame_id,
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
            ),
        ]
    )
