"""
lora_uplink_node
----------------
/odor/sample 의 최신 샘플을 period_s 마다 LoRa 요약 패킷(type 0x03, 53 바이트)으로 E220 에 써서 지상국으로 보낸다.

- E220-900T22D 는 투명 모드(M0 = M1 = 0)로 둔다. 채널, 공중 속도는 지상국 수신 모듈과 같게 미리 설정한다.
- LoRa 는 대역폭이 작고 수신 확인이 없다. 그래서 놓친 패킷을 다시 보내지 않고 "가장 최근 값"만 보낸다.
  1 Hz 원본 전체는 csv_logger_node 가 Pi 에 남긴다 (착륙 후 회수).
- 새 샘플이 없으면 보내지 않는다 (같은 값을 되풀이하지 않음).
- 포트를 못 열거나 쓰다가 끊기면 5 초마다 다시 연다. 그동안에도 노드는 죽지 않는다.

파라미터: port, baud, period_s, station (mob_NN 의 NN. 지상국에서 mob_NN / octo_NN 으로 풀린다)
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from odor_msgs.msg import OdorSample

from .lora_mobile import encode_mobile, sample_fields

PORT = "/dev/ttyUSB0"     # USB-TTL(CP2102). Pi5 추가 UART 면 /dev/ttyAMA2 등 (README 3장)
BAUD = 9600               # E220 UART 기본값. 모듈 설정과 같아야 함
PERIOD_S = 2.0
STATION = 1
REOPEN_S = 5.0


class LoraUplinkNode(Node):
    def __init__(self):
        super().__init__("lora_uplink_node")
        self.port = self.declare_parameter("port", PORT).value
        self.baud = int(self.declare_parameter("baud", BAUD).value)
        period = float(self.declare_parameter("period_s", PERIOD_S).value)
        self.station = int(self.declare_parameter("station", STATION).value)
        if not 1 <= self.station <= 99:
            raise ValueError("station 은 1~99")

        self.ser = None
        self.last_open_try = -1e9
        self.latest = None
        self.seq = 0
        self.sent = 0
        self.errors = 0

        self.create_subscription(OdorSample, "/odor/sample", self.on_sample, 10)
        self.create_timer(period, self.send_latest)
        self.create_timer(60.0, self.report)
        self.open_port()
        self.get_logger().info(f"lora_uplink_node 시작: {self.port} @ {self.baud}, {period:g} 초마다, mob_{self.station:02d}")

    def open_port(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_open_try < REOPEN_S:
            return
        self.last_open_try = now
        try:
            import serial  # pyserial. 시험 환경에서 없어도 모듈은 import 되게
            self.ser = serial.Serial(self.port, self.baud, timeout=0, write_timeout=1.0)
            self.get_logger().info(f"E220 포트 열림: {self.port}")
        except Exception as e:  # noqa: BLE001 - 포트 문제는 모두 같은 방식(재시도)으로 다룬다
            self.ser = None
            self.get_logger().warn(f"E220 포트를 못 엶 ({e}). {REOPEN_S:g} 초 뒤 다시", throttle_duration_sec=30.0)

    def on_sample(self, msg: OdorSample):
        self.latest = msg

    def send_latest(self):
        if self.latest is None:
            return
        if self.ser is None:
            self.open_port()
            if self.ser is None:
                return
        fields = sample_fields(self.latest)
        frame = encode_mobile(self.station, self.seq, fields.pop("epoch_s"), fields)
        try:
            self.ser.write(frame)
        except Exception as e:  # noqa: BLE001
            self.errors += 1
            self.get_logger().warn(f"E220 쓰기 실패 ({e}). 포트를 다시 연다")
            try:
                self.ser.close()
            except Exception:  # noqa: BLE001
                pass
            self.ser = None
            return
        self.latest = None
        self.seq = (self.seq + 1) & 0xFFFF
        self.sent += 1

    def report(self):
        self.get_logger().info(f"LoRa 송신 {self.sent} 건, 오류 {self.errors} 건, 포트 {'열림' if self.ser else '닫힘'}")

    def destroy_node(self):
        if self.ser is not None:
            self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = LoraUplinkNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
