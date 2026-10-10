"""
odor_sample_node
----------------
/drone/state (10 Hz) 와 /odor/gas (1 Hz) 를 구독해서
가스 측정값이 올 때마다 "그 순간의 드론 상태"를 붙여 OdorSample 로 /odor/sample 에 발행한다.
= 지오태깅.

방식: 드론 상태는 최신값을 들고 있다가, 가스 값이 들어오는 순간 붙인다.
(가스 1초 주기 대비 드론 상태 0.1초 주기라 시간 오차는 최대 0.1초)

v0.2 변경
  - drone_age_s: 붙인 드론 상태가 가스 측정 시각보다 몇 초 앞선 것인지. 2 초를 넘으면 위치를 믿지 말 것
  - 드론 상태를 아직 하나도 못 받았어도 샘플을 버리지 않는다.
    위치가 NaN 인 빈 상태를 붙이고 drone_age_s = NaN (JSON null) 으로 보낸다.
    (FC 링크가 없어도 가스 기록은 남도록. 예전에는 이 경우 샘플이 사라졌다)
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from odor_msgs.msg import DroneState, GasReading, OdorSample

DRONE_ID = "octo_01"
STALE_SEC = 2.0   # 드론 상태가 이 시간 이상 오래되면 FC 링크 끊김으로 보고 경고
NAN = float("nan")


def empty_drone(drone_id: str) -> DroneState:
    d = DroneState()
    d.header.frame_id = drone_id
    d.latitude = d.longitude = NAN
    d.altitude_msl = d.altitude_rel = d.heading_deg = NAN
    d.battery_voltage = d.battery_current = NAN
    d.battery_percent = -1.0
    return d


class OdorSampleNode(Node):
    def __init__(self):
        super().__init__("odor_sample_node")
        self.drone_id = self.declare_parameter("drone_id", DRONE_ID).value
        self.latest_drone = None

        self.create_subscription(DroneState, "/drone/state", self.on_drone, 10)
        self.create_subscription(GasReading, "/odor/gas", self.on_gas, 10)
        self.pub = self.create_publisher(OdorSample, "/odor/sample", 10)

        self.get_logger().info("odor_sample_node 시작")

    def on_drone(self, msg: DroneState):
        self.latest_drone = msg

    def on_gas(self, gas: GasReading):
        sample = OdorSample()
        sample.header.stamp = gas.header.stamp
        sample.gas = gas

        if self.latest_drone is None:
            self.get_logger().warn("드론 상태를 아직 못 받음. 위치 없이 기록", throttle_duration_sec=5.0)
            sample.header.frame_id = self.drone_id
            sample.drone = empty_drone(self.drone_id)
            sample.drone.header.stamp = gas.header.stamp
            sample.drone_age_s = NAN
        else:
            age = (Time.from_msg(gas.header.stamp) - Time.from_msg(self.latest_drone.header.stamp)).nanoseconds * 1e-9
            if age > STALE_SEC:
                self.get_logger().warn(f"드론 상태가 {age:.1f}초 전 값 (FC 링크 확인)", throttle_duration_sec=5.0)
            sample.header.frame_id = self.latest_drone.header.frame_id   # 드론 ID
            sample.drone = self.latest_drone
            sample.drone_age_s = float(age) if math.isfinite(age) else NAN
        self.pub.publish(sample)


def main():
    rclpy.init()
    node = OdorSampleNode()
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
