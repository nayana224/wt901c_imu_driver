import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription(
        [
            # 1. IMU 드라이버 실행 (파라미터 주입)
            Node(
                package="wt901c_driver",
                executable="imu_driver",
                name="imu_test_node",
                output="screen",
                parameters=[{"port": "/dev/ttyUSB0", "baudrate": 115200}],
            ),
            # 2. 테스트용 가상 좌표계 설정 (world -> imu_link)
            # 센서가 세계의 중심에 있다고 가정하여 시각화 정렬
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="test_static_tf",
                arguments=["0", "0", "0", "0", "0", "0", "world", "imu_link"],
            ),
            # 3. RViz2 자동 실행
            ExecuteProcess(cmd=["rviz2"], output="screen"),
        ]
    )
