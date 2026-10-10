"""
실제 기체용 런치: mavros + fc_state_node + gas_i2c_node + odor_sample_node + csv_logger_node

    ros2 launch odor_drone real.launch.py                      # 기본: FC 는 mavros, 가스는 I2C 직결
    ros2 launch odor_drone real.launch.py fc:=fake             # FC 없이 책상에서 센서만 시험
    ros2 launch odor_drone real.launch.py gas:=serial          # 아두이노 센서 모듈(시리얼 한 줄 방식)
    ros2 launch odor_drone real.launch.py fcu_url:=serial:///dev/ttyACM0:115200

Pixhawk 연결 포트는 fcu_url 로 바꾼다.
  - RPi5 GPIO UART(핀 8, 10) <-> Pixhawk TELEM2 : serial:///dev/ttyAMA0:921600
  - USB 케이블로 직결                            : serial:///dev/ttyACM0:115200
펌웨어가 PX4 면 mavros_launch:=px4.launch
센서 주소, 채널, ID 는 config/mob_01.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

GAS_NODES = {"i2c": "gas_i2c_node", "serial": "gas_sensor_node", "fake": "fake_gas_node"}
FC_NODES = {"mavros": "fc_state_node", "fake": "fake_fc_node"}


def _nodes(context):
    fc = LaunchConfiguration("fc").perform(context)
    gas = LaunchConfiguration("gas").perform(context)
    params = LaunchConfiguration("params").perform(context)
    if fc not in FC_NODES or gas not in GAS_NODES:
        raise RuntimeError(f"fc 는 {list(FC_NODES)}, gas 는 {list(GAS_NODES)} 중 하나")

    def node(name):
        return Node(package="odor_drone", executable=name, name=name, output="screen", parameters=[params])

    actions = []
    if fc == "mavros":
        actions.append(IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(get_package_share_directory("mavros"), "launch",
                                                    LaunchConfiguration("mavros_launch").perform(context))),
            launch_arguments={"fcu_url": LaunchConfiguration("fcu_url").perform(context)}.items(),
        ))
    actions += [
        node(FC_NODES[fc]),         # /mavros/*  -> /drone/state
        node(GAS_NODES[gas]),       # 센서       -> /odor/gas, /health/mob_01
        node("odor_sample_node"),   # 위 둘      -> /odor/sample
        node("csv_logger_node"),    # /odor/sample -> CSV, JSONL
    ]
    return actions


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("odor_drone"), "config", "mob_01.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("fc", default_value="mavros", description="mavros | fake"),
        DeclareLaunchArgument("gas", default_value="i2c", description="i2c | serial | fake"),
        DeclareLaunchArgument("fcu_url", default_value="serial:///dev/ttyAMA0:921600"),
        DeclareLaunchArgument("mavros_launch", default_value="apm.launch", description="ArduPilot. PX4 면 px4.launch"),
        DeclareLaunchArgument("params", default_value=default_params, description="파라미터 파일"),
        OpaqueFunction(function=_nodes),
    ])
