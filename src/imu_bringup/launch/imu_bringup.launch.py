import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 패키지 경로 설정
    pkg_share_path = get_package_share_directory("imu_bringup")  # 패키지명 확인 필요

    return LaunchDescription(
        [
            # IMU 드라이버 노드 (SLAM/Nav2 연동용)
            Node(
                package="imu_bringup",
                executable="imu_driver",
                name="imu_driver_node",
                output="screen",
                parameters=[
                    {
                        "port": "/dev/ttyUSB0",
                        "baudrate": 115200,
                    }
                ],
                # SLAM 연동 시 안정성을 위해 토픽 리매핑 가능
                remappings=[("/imu/data", "/imu/data_raw")],
            ),
        ]
    )
