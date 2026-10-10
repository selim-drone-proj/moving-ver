"""
fake_fc_node
------------
Pixhawk 와 mavros 없이 파이프라인을 돌려보기 위한 가짜 드론 상태.
fc_state_node 와 똑같이 /drone/state 에 DroneState 를 10 Hz 발행한다.
내용: 중심 좌표 주위를 반지름 30 m 원으로 도는 비행 + 천천히 떨어지는 배터리.

파라미터: drone_id (octo_01), center_lat, center_lon, radius_m, alt_rel_m, lap_s
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from odor_msgs.msg import DroneState

DRONE_ID = "octo_01"
PUBLISH_HZ = 10.0

CENTER_LAT = 35.8242   # 시험 현장 좌표로 바꿀 것
CENTER_LON = 127.1480
RADIUS_M = 30.0
ALT_REL_M = 20.0
LAP_SEC = 60.0         # 한 바퀴 도는 시간


class FakeFcNode(Node):
    def __init__(self):
        super().__init__("fake_fc_node")
        self.drone_id = self.declare_parameter("drone_id", DRONE_ID).value
        self.center_lat = self.declare_parameter("center_lat", CENTER_LAT).value
        self.center_lon = self.declare_parameter("center_lon", CENTER_LON).value
        self.radius_m = self.declare_parameter("radius_m", RADIUS_M).value
        self.alt_rel_m = self.declare_parameter("alt_rel_m", ALT_REL_M).value
        self.lap_s = self.declare_parameter("lap_s", LAP_SEC).value
        self.pub = self.create_publisher(DroneState, "/drone/state", 10)
        self.t = 0.0
        self.create_timer(1.0 / PUBLISH_HZ, self.publish)
        self.get_logger().info("fake_fc_node 시작 (가짜 드론 상태 발행)")

    def publish(self):
        self.t += 1.0 / PUBLISH_HZ
        ang = 2.0 * math.pi * self.t / self.lap_s          # 원 위 각도 (rad)

        # 위도 1도 ~ 111 km, 경도 1도 ~ 111 km * cos(위도)
        dlat = self.radius_m * math.cos(ang) / 111_000.0
        dlon = self.radius_m * math.sin(ang) / (111_000.0 * math.cos(math.radians(self.center_lat)))

        m = DroneState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.drone_id
        m.armed = True
        m.mode = "AUTO"
        m.in_air = True
        m.latitude = self.center_lat + dlat
        m.longitude = self.center_lon + dlon
        m.altitude_rel = self.alt_rel_m
        m.altitude_msl = self.alt_rel_m + 40.0
        m.heading_deg = (math.degrees(ang) + 90.0) % 360.0   # 원 접선 방향
        m.yaw_deg = 90.0 - m.heading_deg                     # ENU 로 환산
        m.gps_fix_type = 3
        m.gps_satellites = 14
        m.ground_speed_mps = 2.0 * math.pi * self.radius_m / self.lap_s
        m.battery_voltage = 16.8 - 0.002 * self.t          # 4S 기준, 서서히 감소
        m.battery_current = 25.0
        m.battery_percent = max(0.0, 100.0 - 0.05 * self.t)
        self.pub.publish(m)


def main():
    rclpy.init()
    node = FakeFcNode()
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
