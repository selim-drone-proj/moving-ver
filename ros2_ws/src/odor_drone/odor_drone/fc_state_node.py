"""
fc_state_node
-------------
Pixhawk 데이터를 MAVROS 토픽(/mavros/*)으로 받아서
필요한 것만 골라 DroneState 하나로 묶어 /drone/state 로 10 Hz 발행한다.

동작 방식 (전부 pub/sub):
  1. /mavros/* 토픽 9개를 구독한다.
  2. 콜백에서는 self.state 에 최신값만 덮어쓴다.
  3. 타이머가 10 Hz 로 self.state 를 그대로 발행한다.

v0.2 변경
  - 드론 ID 를 파라미터로 (기본 octo_01. ROS 토픽 이름 규칙에 맞게 하이픈 대신 밑줄)
  - 위치를 아직 모르면 0.0 대신 NaN (GPS fix 없음, 아직 수신 전). 0, 0 좌표가 기록되는 것을 막는다
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu, BatteryState
from mavros_msgs.msg import State, ExtendedState, VfrHud, GPSRAW
from odor_msgs.msg import DroneState

DRONE_ID = "octo_01"   # 기본 기체 식별자 (파라미터 drone_id). OdorSample 의 frame_id 로도 쓰인다
PUBLISH_HZ = 10.0
NAN = float("nan")


def quat_to_euler_deg(q):
    """쿼터니언 -> (roll, pitch, yaw) [deg]"""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class FcStateNode(Node):
    def __init__(self):
        super().__init__("fc_state_node")

        drone_id = self.declare_parameter("drone_id", DRONE_ID).value
        self.state = DroneState()
        self.state.header.frame_id = drone_id
        self.state.latitude = NAN        # 위치를 받기 전에는 NaN (JSON null)
        self.state.longitude = NAN
        self.state.altitude_msl = NAN
        self.state.altitude_rel = NAN
        self.state.battery_percent = -1.0

        # mavros 는 센서 토픽을 best-effort 로 발행하므로 구독도 sensor QoS 로 맞춘다
        qos = qos_profile_sensor_data
        self.create_subscription(State,        "/mavros/state",                        self.on_state,     qos)
        self.create_subscription(ExtendedState,"/mavros/extended_state",               self.on_ext_state, qos)
        self.create_subscription(NavSatFix,    "/mavros/global_position/global",       self.on_gps,       qos)
        self.create_subscription(Float64,      "/mavros/global_position/rel_alt",      self.on_rel_alt,   qos)
        self.create_subscription(Float64,      "/mavros/global_position/compass_hdg",  self.on_heading,   qos)
        self.create_subscription(GPSRAW,       "/mavros/gpsstatus/gps1/raw",           self.on_gps_raw,   qos)
        self.create_subscription(Imu,          "/mavros/imu/data",                     self.on_imu,       qos)
        self.create_subscription(VfrHud,       "/mavros/vfr_hud",                      self.on_vfr_hud,   qos)
        self.create_subscription(BatteryState, "/mavros/battery",                      self.on_battery,   qos)

        self.pub = self.create_publisher(DroneState, "/drone/state", 10)
        self.create_timer(1.0 / PUBLISH_HZ, self.publish)

        self.fc_connected = False
        self.get_logger().info(f"fc_state_node 시작 ({drone_id}). /mavros/* 대기 중")

    # ---------- 콜백: 최신값만 저장 ----------
    def on_state(self, msg: State):
        self.state.armed = msg.armed
        self.state.mode = msg.mode
        if msg.connected != self.fc_connected:
            self.fc_connected = msg.connected
            self.get_logger().info(f"FC 연결: {msg.connected}")

    def on_ext_state(self, msg: ExtendedState):
        self.state.in_air = msg.landed_state in (ExtendedState.LANDED_STATE_IN_AIR,
                                                 ExtendedState.LANDED_STATE_TAKEOFF,
                                                 ExtendedState.LANDED_STATE_LANDING)

    def on_gps(self, msg: NavSatFix):
        if msg.status.status < 0:          # STATUS_NO_FIX: FC 가 0, 0 을 보낼 수 있으므로 버린다
            self.state.latitude = self.state.longitude = self.state.altitude_msl = NAN
            return
        self.state.latitude = msg.latitude
        self.state.longitude = msg.longitude
        self.state.altitude_msl = float(msg.altitude)

    def on_rel_alt(self, msg: Float64):
        self.state.altitude_rel = float(msg.data)

    def on_heading(self, msg: Float64):
        self.state.heading_deg = float(msg.data)

    def on_gps_raw(self, msg: GPSRAW):
        self.state.gps_fix_type = msg.fix_type
        self.state.gps_satellites = msg.satellites_visible

    def on_imu(self, msg: Imu):
        roll, pitch, yaw = quat_to_euler_deg(msg.orientation)
        self.state.roll_deg = roll
        self.state.pitch_deg = pitch
        self.state.yaw_deg = yaw

    def on_vfr_hud(self, msg: VfrHud):
        self.state.ground_speed_mps = msg.groundspeed
        self.state.climb_rate_mps = msg.climb

    def on_battery(self, msg: BatteryState):
        self.state.battery_voltage = msg.voltage
        self.state.battery_current = abs(msg.current)     # ROS 규약상 방전이 음수라 절댓값
        if not math.isnan(msg.percentage):
            self.state.battery_percent = msg.percentage * 100.0   # mavros 는 0~1 로 준다

    # ---------- 타이머: 그대로 발행 ----------
    def publish(self):
        self.state.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.state)


def main():
    rclpy.init()
    node = FcStateNode()
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
