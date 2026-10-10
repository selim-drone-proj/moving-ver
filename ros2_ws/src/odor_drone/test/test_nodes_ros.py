"""
실제 ROS 2(rclpy + 빌드된 odor_msgs)로 하는 시험. ROS 가 없는 PC 에서는 건너뛴다.

    cd ros2_ws && colcon build && source install/setup.bash
    python3 -m pytest -q src/odor_drone/test
"""
import json
import math
import sys
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
odor_msgs = pytest.importorskip("odor_msgs.msg")

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
MOVING = PKG.parents[2]
sys.path.insert(0, str(MOVING / "tools"))

from builtin_interfaces.msg import Time  # noqa: E402
from odor_msgs.msg import DroneState, FireStatus, GasReading, OdorSample, StationSample, DeviceHealth  # noqa: E402

from odor_drone import csv_logger_node, odor_json  # noqa: E402
from odor_drone.fake_values import fake_gas  # noqa: E402
from odor_drone.odor_sample_node import OdorSampleNode  # noqa: E402

CLASSES = {"drone_state": DroneState, "gas_reading": GasReading, "odor_sample": OdorSample,
           "station_sample": StationSample, "fire_status": FireStatus, "device_health": DeviceHealth}


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def stamp(t: float) -> Time:
    return Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))


def gas_msg(t: float) -> GasReading:
    g = GasReading()
    g.header.stamp = stamp(t)
    g.header.frame_id = "mob_01"
    for k, v in fake_gas(t).items():
        setattr(g, k, v)
    return g


@pytest.mark.parametrize("path", sorted((MOVING / "interfaces" / "examples").glob("*.json")), ids=lambda p: p.stem)
def test_examples_round_trip_with_real_messages(path):
    rec = json.loads(path.read_text(encoding="utf-8"))
    msg = odor_json.fill_msg(CLASSES[rec["type"]](), rec)
    assert odor_json.msg_to_dict(msg, rec["type"]) == rec


def test_csv_row_matches_columns_and_blanks_nan():
    s = OdorSample()
    s.header.stamp = stamp(1791609153.12)
    s.header.frame_id = "octo_01"
    s.gas = gas_msg(1791609153.12)
    s.drone.latitude = float("nan")
    s.drone_age_s = float("nan")
    row = csv_logger_node.csv_row(s)
    assert len(row) == len(csv_logger_node.COLUMNS)
    cols = dict(zip(csv_logger_node.COLUMNS, row))
    assert cols["time_iso"] == "2026-10-10T05:12:33.120Z"
    assert cols["lat"] == "" and cols["drone_age_s"] == "" and cols["voc_index"] == ""
    assert cols["sensor_id"] == "mob_01" and cols["flags"] == GasReading.FLAG_SIMULATED
    assert csv_logger_node.COLUMNS[:28][-1] == "payload_v"          # v0.1 열은 그대로


def test_odor_sample_node_geotags_and_keeps_gas_without_fc(ros):
    import validate_jsonl

    node = OdorSampleNode()
    helper = rclpy.create_node("test_helper")
    got = []
    helper.create_subscription(OdorSample, "/odor/sample", got.append, 10)
    pub_gas = helper.create_publisher(GasReading, "/odor/gas", 10)
    pub_drone = helper.create_publisher(DroneState, "/drone/state", 10)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(helper)

    def spin_for(cond, timeout=3.0):
        end = time.time() + timeout
        while time.time() < end and not cond():
            ex.spin_once(timeout_sec=0.05)
        return cond()

    spin_for(lambda: pub_gas.get_subscription_count() > 0 and helper.count_publishers("/odor/sample") > 0, 2.0)

    # 1) FC 상태 없이 가스만: 버리지 않고 위치 NaN, drone_age_s NaN
    pub_gas.publish(gas_msg(1000.0))
    assert spin_for(lambda: len(got) >= 1)
    s = got[-1]
    assert math.isnan(s.drone.latitude) and math.isnan(s.drone_age_s) and s.header.frame_id == "octo_01"
    rec = odor_json.msg_to_dict(s, "odor_sample")
    assert rec["drone_age_s"] is None and rec["drone"]["latitude"] is None
    assert validate_jsonl.errors(rec) == []

    # 2) 드론 상태 수신 후: 가스 측정 시각 - 드론 상태 시각
    d = DroneState()
    d.header.stamp = stamp(1000.7)
    d.header.frame_id = "octo_01"
    d.latitude, d.longitude = 35.8242, 127.148
    pub_drone.publish(d)
    spin_for(lambda: node.latest_drone is not None)
    pub_gas.publish(gas_msg(1001.0))
    assert spin_for(lambda: len(got) >= 2)
    s = got[-1]
    assert s.drone_age_s == pytest.approx(0.3, abs=1e-3) and s.drone.latitude == pytest.approx(35.8242)
    assert validate_jsonl.errors(odor_json.msg_to_dict(s, "odor_sample")) == []
    node.destroy_node()
    helper.destroy_node()
