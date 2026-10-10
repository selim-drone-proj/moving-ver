"""
fake_gas_node
-------------
센서 없이 파이프라인을 돌려보기 위한 가짜 가스 값.
gas_i2c_node 와 똑같이 /odor/gas 에 GasReading 을 1 Hz, /health/<device_id> 에 DeviceHealth 를 10 초마다 발행한다.
내용: 배경 농도 + 잡음, 그리고 40초마다 10초짜리 악취 구름(plume) 통과 (fake_values.py).

v0.2: 새 필드(eco2_ppm, pressure_hpa, gas_resistance_ohm, raw, flags)도 채우고
      flags 에 FLAG_SIMULATED(bit11) 를 세운다. 분석할 때 이 비트로 가짜 데이터를 걸러낼 것.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from odor_msgs.msg import DeviceHealth, GasReading

from . import __version__
from .fake_values import fake_gas

SENSOR_ID = "mob_01"
READ_HZ = 1.0


class FakeGasNode(Node):
    def __init__(self):
        super().__init__("fake_gas_node")
        self.device_id = self.declare_parameter("device_id", SENSOR_ID).value
        self.pub = self.create_publisher(GasReading, "/odor/gas", 10)
        self.pub_health = self.create_publisher(DeviceHealth, f"/health/{self.device_id}", 10)
        self.t = 0.0
        self.create_timer(1.0 / READ_HZ, self.publish)
        self.create_timer(10.0, self.publish_health)
        self.get_logger().info(f"fake_gas_node 시작 ({self.device_id}, 가짜 가스 값 발행)")

    def publish(self):
        self.t += 1.0 / READ_HZ
        m = GasReading()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.device_id
        for key, val in fake_gas(self.t).items():
            setattr(m, key, val)
        self.pub.publish(m)

    def publish_health(self):
        h = DeviceHealth()
        h.header.stamp = self.get_clock().now().to_msg()
        h.header.frame_id = self.device_id
        h.state = DeviceHealth.STATE_OK
        h.firmware = f"fake_gas_node {__version__}"
        h.uptime_s = int(self.t)
        h.sensor_ok_mask = 0b11111
        h.supply_v = float("nan")
        h.cpu_temp_c = float("nan")
        h.message = "simulated"
        self.pub_health.publish(h)


def main():
    rclpy.init()
    node = FakeGasNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:   # 종료 중 Ctrl-C 가 한 번 더 들어온 경우 (ros2 launch 종료 때 흔함)
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
