"""
하드웨어 없이 돌리는 런치: fake_fc_node + fake_gas_node + odor_sample_node + csv_logger_node

    ros2 launch odor_drone fake.launch.py

real.launch.py 와 토픽 이름, 메시지 타입이 완전히 같으므로
소프트웨어 쪽은 이걸로 개발하다가 실기체로 바꾸기만 하면 된다.
가짜 가스 값에는 flags 의 FLAG_SIMULATED(bit11) 가 켜져 있다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("odor_drone"), "config", "mob_01.yaml")
    params = LaunchConfiguration("params")

    def node(name):
        return Node(package="odor_drone", executable=name, name=name, output="screen", parameters=[params])

    return LaunchDescription([
        DeclareLaunchArgument("params", default_value=default_params, description="파라미터 파일"),
        node("fake_fc_node"),       # 가짜 드론 상태 -> /drone/state
        node("fake_gas_node"),      # 가짜 가스 값   -> /odor/gas, /health/mob_01
        node("odor_sample_node"),   # 위 둘          -> /odor/sample
        node("csv_logger_node"),    # /odor/sample   -> CSV, JSONL
    ])
