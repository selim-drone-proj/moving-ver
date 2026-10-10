"""
gas_i2c_node
------------
이동형 센서 4종(Gravity NH3, Gravity H2S, SGP30, BME688)을 Pi5 의 I2C 로 직접 읽어
  /odor/gas           odor_msgs/GasReading   1 Hz
  /health/<device_id> odor_msgs/DeviceHealth 10 초마다
로 발행한다. (예전 gas_sensor_node 는 아두이노 시리얼 방식. 그것도 남겨 두었다)

    ros2 run odor_drone gas_i2c_node --ros-args --params-file config/mob_01.yaml

배선, 주소, 채널은 파라미터로 바꾼다 (config/mob_01.yaml 설명 참고).
센서 하나가 빠져도 나머지 값으로 계속 발행하고, 빠진 값은 NaN + flags 로 표시한다.
읽기는 gas_reader.GasReader 가 하고 이 노드는 ROS 메시지로 옮기기만 한다.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from odor_msgs.msg import DeviceHealth, GasReading

from . import __version__, system_info
from .drivers.bme688 import BME688
from .drivers.dfrobot_gas import GravityGas
from .drivers.i2c_bus import SMBus2Bus
from .drivers.sgp30 import SGP30
from .drivers.tca9548a import TCA9548A
from .gas_reader import GasReader, ReaderConfig

PARAMS = [
    ("device_id", "mob_01"),
    ("i2c_bus", 1),                 # /dev/i2c-1 (40핀 3번 SDA, 5번 SCL)
    ("rate_hz", 1.0),
    ("health_period_s", 10.0),
    ("tca_address", 0x70),          # TCA9548A 주소. -1 = 멀티플렉서 없이 직결 (채널 무시)
    ("nh3_enabled", True), ("nh3_address", 0x74), ("nh3_channel", 0),
    ("h2s_enabled", True), ("h2s_address", 0x74), ("h2s_channel", 1),
    ("sgp30_enabled", True), ("sgp30_channel", 2),
    ("bme688_enabled", True), ("bme688_address", 0x76), ("bme688_channel", 3),
    ("gravity_settle_s", 0.1),      # Gravity 명령 후 응답까지 기다리는 시간
    ("warmup_s", 180.0),            # 켠 뒤 이 시간 동안 FLAG_WARMUP
    ("temp_comp", ""),              # 제조사 온도 보정 적용 가스. 예: "NH3" 또는 "NH3,H2S". 기본은 없음
    ("retry_s", 30.0),
    ("simulate_bus", False),        # 개발용: 실제 I2C 대신 가짜 장치(drivers/fakes.py). flags 에 SIMULATED
]


def simulated_bus(p: dict):
    """가짜 장치를 붙인 버스와 가짜 BME688. 값은 fake_values 의 40 초 주기 악취 구름을 따른다"""
    import time

    from .drivers.fakes import FakeBME688, FakeGravity, FakeMuxBus, FakeSGP30
    from .fake_values import plume

    t0 = time.monotonic()

    def pl():
        return plume(time.monotonic() - t0)

    bus = FakeMuxBus(p["tca_address"] if p["tca_address"] >= 0 else 0x70)
    devices = [("nh3", p["nh3_address"], FakeGravity("NH3", lambda: 0.5 + 6.0 * pl())),
               ("h2s", p["h2s_address"], FakeGravity("H2S", lambda: 0.02 + 0.8 * pl())),
               ("sgp30", 0x58, FakeSGP30(tvoc=lambda: 120 + 900 * pl()))]
    for name, addr, dev in devices:
        if p["tca_address"] >= 0:
            bus.attach_ch(p[f"{name}_channel"], addr, dev)
        else:
            bus.attach(addr, dev)
    return bus, FakeBME688(t=27.0, rh=60.0, p=1008.5, gas_ohm=lambda: 60000.0 / (1.0 + 3.0 * pl()))


class GasI2cNode(Node):
    def __init__(self):
        super().__init__("gas_i2c_node")
        self.declare_parameters("", PARAMS)
        p = {name: self.get_parameter(name).value for name, _ in PARAMS}
        self.device_id = p["device_id"]

        self.simulated = p["simulate_bus"]
        fake_bme = None
        if self.simulated:
            self.bus, fake_bme = simulated_bus(p)
            self.get_logger().warn("simulate_bus: 가짜 I2C 장치로 동작 (flags 에 SIMULATED)")
        else:
            try:
                self.bus = SMBus2Bus(p["i2c_bus"])
            except (FileNotFoundError, PermissionError) as e:
                self.get_logger().fatal(f"/dev/i2c-{p['i2c_bus']} 를 열 수 없음 ({e}). "
                                        "sudo raspi-config 에서 I2C 를 켜고, 사용자를 i2c 그룹에 넣을 것")
                raise SystemExit(1)
        mux = TCA9548A(self.bus, p["tca_address"]) if p["tca_address"] >= 0 else None

        def on(ch):
            return mux.channel(ch) if mux else self.bus

        settle = p["gravity_settle_s"]
        cfg = ReaderConfig(warmup_s=p["warmup_s"], retry_s=p["retry_s"],
                           temp_comp=tuple(g.strip().upper() for g in p["temp_comp"].split(",") if g.strip()))
        self.reader = GasReader(
            nh3=GravityGas(on(p["nh3_channel"]), p["nh3_address"], "NH3", settle) if p["nh3_enabled"] else None,
            h2s=GravityGas(on(p["h2s_channel"]), p["h2s_address"], "H2S", settle) if p["h2s_enabled"] else None,
            sgp30=SGP30(on(p["sgp30_channel"])) if p["sgp30_enabled"] else None,
            bme688=(fake_bme or BME688(on(p["bme688_channel"]), p["bme688_address"])) if p["bme688_enabled"] else None,
            mux=mux, cfg=cfg, clock_synced=system_info.clock_synced,
            log=lambda m: self.get_logger().info(m))

        mask = self.reader.begin()
        self.get_logger().info(f"{self.device_id} 센서 시작. sensor_ok_mask=0b{mask:05b} "
                               f"(bit0 NH3, bit1 H2S, bit2 SGP30, bit3 BME688, bit4 TCA9548A)")
        for slot in self.reader.slots.values():
            if not slot.ok:
                self.get_logger().warn(f"{slot.name} 연결 실패: {slot.last_error}")

        self.pub = self.create_publisher(GasReading, "/odor/gas", 10)
        self.pub_health = self.create_publisher(DeviceHealth, f"/health/{self.device_id}", 10)
        self.create_timer(1.0 / p["rate_hz"], self.read_and_publish)
        self.create_timer(p["health_period_s"], self.publish_health)
        self.publish_health()

    def read_and_publish(self):
        values = self.reader.read()
        msg = GasReading()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.device_id
        for key, val in values.items():
            setattr(msg, key, val)
        if self.simulated:
            msg.flags |= GasReading.FLAG_SIMULATED
        self.pub.publish(msg)

    def publish_health(self):
        r = self.reader
        h = DeviceHealth()
        h.header.stamp = self.get_clock().now().to_msg()
        h.header.frame_id = self.device_id
        h.state = r.state
        h.firmware = f"odor_drone {__version__}" + (" (simulate_bus)" if self.simulated else "")
        h.uptime_s = system_info.uptime_s()
        h.sensor_ok_mask = r.sensor_ok_mask
        h.error_count = min(r.error_count, 65535)
        h.supply_v = system_info.supply_v()
        h.cpu_temp_c = system_info.cpu_temp_c()
        h.rssi_dbm = 0
        h.message = r.last_error[:200]
        self.pub_health.publish(h)
        if h.state >= DeviceHealth.STATE_DEGRADED:
            self.get_logger().warn(f"상태 {h.state} mask=0b{h.sensor_ok_mask:05b} {h.message}",
                                   throttle_duration_sec=60.0)

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = GasI2cNode()
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

