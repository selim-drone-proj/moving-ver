"""
csv_logger_node
---------------
/odor/sample 을 구독해서 한 줄씩 CSV 로 저장한다 (RPi 로컬 백업).
노지에서 무선이 끊겨도 비행 데이터는 여기 남는다. 착륙 후 파일만 가져가면 된다.

파일 (노드 시작 시각 기준, 실행마다 새 파일, 파일 이름의 시각은 UTC)
  LOG_DIR/odor_YYYYmmdd_HHMMSSZ.csv    사람이 보기 쉬운 표
  LOG_DIR/odor_YYYYmmdd_HHMMSSZ.jsonl  규격 v0.2 의 odor_sample 기록 (한 줄에 하나, 스키마로 검사 가능)

v0.2 변경 (기존 열은 이름, 순서 그대로)
  - time_iso 를 현지 시각(시간대 표시 없음)에서 UTC (끝에 Z) 로 바꿈. time_unix 는 그대로
  - 값이 없으면(NaN) "nan" 대신 빈 칸
  - 맨 뒤에 열 추가: drone_age_s, sensor_id, eco2_ppm ... flags (COLUMNS 참고)
  - JSONL 파일을 같이 남긴다 (write_jsonl 파라미터로 끌 수 있음)
"""

import csv
import json
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from odor_msgs.msg import OdorSample

from .odor_json import iso_utc, msg_to_dict

LOG_DIR = "~/odor_logs"

COLUMNS = [
    # ---- v0.1 ----
    "time_unix", "time_iso", "drone_id",
    "armed", "mode", "in_air",
    "lat", "lon", "alt_msl_m", "alt_rel_m", "heading_deg", "gps_fix", "gps_sats",
    "roll_deg", "pitch_deg", "yaw_deg", "ground_speed_mps", "climb_rate_mps",
    "batt_v", "batt_a", "batt_pct",
    "h2s_ppm", "nh3_ppm", "tvoc_ppb", "temp_c", "hum_pct", "pump_on", "payload_v",
    # ---- v0.2 추가 (고정형 CSV 와 같은 이름) ----
    "drone_age_s", "sensor_id", "eco2_ppm", "voc_index", "nox_index", "pressure_hpa", "gas_res_ohm",
    "pm1_0", "pm2_5", "pm10", "nh3_raw", "h2s_raw", "flags",
]


def f(v, nd: int) -> str:
    """숫자 -> 문자열. NaN, inf 는 빈 칸"""
    v = float(v)
    return "" if not math.isfinite(v) else f"{v:.{nd}f}"


def csv_row(s: OdorSample) -> list:
    d, g = s.drone, s.gas
    t = s.header.stamp.sec + s.header.stamp.nanosec * 1e-9
    return [
        f"{t:.3f}", iso_utc(t), s.header.frame_id,
        int(d.armed), d.mode, int(d.in_air),
        f(d.latitude, 7), f(d.longitude, 7), f(d.altitude_msl, 2), f(d.altitude_rel, 2),
        f(d.heading_deg, 1), d.gps_fix_type, d.gps_satellites,
        f(d.roll_deg, 1), f(d.pitch_deg, 1), f(d.yaw_deg, 1),
        f(d.ground_speed_mps, 2), f(d.climb_rate_mps, 2),
        f(d.battery_voltage, 2), f(d.battery_current, 2), f(d.battery_percent, 0),
        f(g.h2s_ppm, 3), f(g.nh3_ppm, 3), f(g.tvoc_ppb, 0),
        f(g.temperature_c, 1), f(g.humidity_pct, 1), int(g.pump_on), f(g.payload_voltage, 2),
        f(s.drone_age_s, 2), g.header.frame_id, f(g.eco2_ppm, 0), f(g.voc_index, 0), f(g.nox_index, 0),
        f(g.pressure_hpa, 1), f(g.gas_resistance_ohm, 0),
        f(g.pm1_0_ugm3, 0), f(g.pm2_5_ugm3, 0), f(g.pm10_ugm3, 0),
        f(g.nh3_raw, 3), f(g.h2s_raw, 3), g.flags,
    ]


class CsvLoggerNode(Node):
    def __init__(self):
        super().__init__("csv_logger_node")
        log_dir = os.path.expanduser(self.declare_parameter("log_dir", LOG_DIR).value)
        write_jsonl = self.declare_parameter("write_jsonl", True).value

        os.makedirs(log_dir, exist_ok=True)
        base = os.path.join(log_dir, time.strftime("odor_%Y%m%d_%H%M%SZ", time.gmtime()))
        self.file = open(base + ".csv", "w", newline="")
        self.writer = csv.writer(self.file, lineterminator="\n")
        self.writer.writerow(COLUMNS)
        self.file.flush()
        self.jsonl = open(base + ".jsonl", "w") if write_jsonl else None

        self.create_subscription(OdorSample, "/odor/sample", self.on_sample, 10)
        self.count = 0
        self.get_logger().info(f"기록 시작: {base}.csv" + (" (+ .jsonl)" if self.jsonl else ""))

    def on_sample(self, s: OdorSample):
        self.writer.writerow(csv_row(s))
        self.file.flush()   # 매 줄 저장. 전원이 갑자기 꺼져도 직전 줄까지 남는다
        if self.jsonl:
            self.jsonl.write(json.dumps(msg_to_dict(s, "odor_sample"), ensure_ascii=False) + "\n")
            self.jsonl.flush()

        self.count += 1
        if self.count % 60 == 0:
            self.get_logger().info(f"{self.count} 줄 기록됨")

    def destroy_node(self):
        self.file.close()
        if self.jsonl:
            self.jsonl.close()
        super().destroy_node()


def main():
    rclpy.init()  # ROS 2 통신 계층 초기화
    node = CsvLoggerNode()  # __init__ 이 한 번 실행됨
    try:
        rclpy.spin(node)  # 프로그램이 멈춰서 메시지를 기다림
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
