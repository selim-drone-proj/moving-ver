"""
gas_sensor_node
---------------
악취 센서 모듈(아두이노 등 MCU)이 시리얼로 보내는 한 줄을 읽어
GasReading 으로 /odor/gas 에 1 Hz 발행한다.

시리얼 한 줄 형식 (MCU 쪽 arduino/gas_module.ino 참고):
    h2s_ppm,nh3_ppm,tvoc_ppb,temp_c,hum_pct,pump_on,payload_v\n
    예)  0.12,1.80,240,27.3,61.5,1,15.9

센서를 RPi I2C/ADC 에 직접 붙이는 구조로 바꾸면 read_line() 만 교체하면 된다.
-> v0.2 에서 그 구조가 gas_i2c_node 다. 이 노드는 아두이노 모듈을 쓸 때를 위해 남겨 둔다.
   (real.launch.py gas:=serial)

v0.2 변경
  - 장비 ID 기본값 mob_01 (파라미터 device_id), 포트와 속도도 파라미터 (port, baud)
  - 시리얼 줄에 없는 v0.2 필드는 NaN, flags 에 pump_on 과 시각 동기 여부만 표시
"""

import math

import serial

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from odor_msgs.msg import GasReading

from .system_info import clock_synced

SENSOR_ID = "mob_01"
NAN = float("nan")
NEW_FIELDS = ("eco2_ppm", "voc_index", "nox_index", "pressure_hpa", "gas_resistance_ohm",
              "pm1_0_ugm3", "pm2_5_ugm3", "pm10_ugm3", "nh3_raw", "h2s_raw")
SERIAL_PORT = "/dev/ttyUSB0"   # 아두이노 나노 등 USB 시리얼. 보드에 따라 /dev/ttyACM0
BAUD = 115200
READ_HZ = 1.0


class GasSensorNode(Node):
    def __init__(self):
        super().__init__("gas_sensor_node")
        self.device_id = self.declare_parameter("device_id", SENSOR_ID).value
        port = self.declare_parameter("port", SERIAL_PORT).value
        baud = self.declare_parameter("baud", BAUD).value
        self.pub = self.create_publisher(GasReading, "/odor/gas", 10)

        self.ser = serial.Serial(port, baud, timeout=0.5)
        self.ser.reset_input_buffer()
        self.get_logger().info(f"센서 시리얼 열림: {port} @ {baud}")

        self.create_timer(1.0 / READ_HZ, self.read_and_publish)

    def read_line(self):
        """버퍼에 쌓인 줄 중 가장 최신 한 줄만 돌려준다 (없으면 None)."""
        latest = None
        while self.ser.in_waiting:
            line = self.ser.readline().decode("ascii", errors="ignore").strip()
            if line:
                latest = line
        return latest

    def read_and_publish(self):
        line = self.read_line()
        if line is None:
            self.get_logger().warn("센서 데이터 없음", throttle_duration_sec=5.0)
            return

        parts = line.split(",")
        if len(parts) != 7:
            self.get_logger().warn(f"형식 안 맞는 줄 무시: {line}")
            return
        try:
            h2s, nh3, tvoc, temp, hum, pump, volt = [float(p) for p in parts]
        except ValueError:
            self.get_logger().warn(f"숫자 변환 실패, 무시: {line}")
            return

        msg = GasReading()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.device_id
        msg.h2s_ppm = h2s
        msg.nh3_ppm = nh3
        msg.tvoc_ppb = tvoc
        msg.temperature_c = temp
        msg.humidity_pct = hum
        msg.pump_on = pump > 0.5
        msg.payload_voltage = volt
        for name in NEW_FIELDS:
            setattr(msg, name, NAN)
        msg.flags = (GasReading.FLAG_PUMP_ON if msg.pump_on else 0) | \
                    (0 if clock_synced() else GasReading.FLAG_TIME_UNSYNCED)
        for name, bit in (("nh3_ppm", GasReading.FLAG_NH3_INVALID), ("h2s_ppm", GasReading.FLAG_H2S_INVALID)):
            if math.isnan(getattr(msg, name)):
                msg.flags |= bit
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GasSensorNode()
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
