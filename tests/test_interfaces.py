"""규격 자체의 검증: .msg, JSON Schema, 예시, JSON<->ROS 변환이 서로 맞는가."""
import copy
import json
import math
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "interfaces" / "python"))
sys.path.insert(0, str(ROOT / "tests"))

import msgspec  # noqa: E402
import odor_json  # noqa: E402
import validate_jsonl  # noqa: E402
from fake_rosidl import build_classes  # noqa: E402

EXAMPLES = sorted((ROOT / "interfaces" / "examples").glob("*.json"))
CLASS_OF = {"drone_state": "DroneState", "gas_reading": "GasReading", "odor_sample": "OdorSample",
            "station_sample": "StationSample", "fire_status": "FireStatus", "device_health": "DeviceHealth"}


def load(p):
    return json.loads(p.read_text(encoding="utf-8"))


def test_msg_files_follow_ros2_rules():
    assert msgspec.validate_names(msgspec.load_all()) == []


def test_schemas_are_generated_from_msg():
    r = subprocess.run([sys.executable, str(ROOT / "tools" / "gen_schemas.py"), "--check"], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout


def test_v01_fields_unchanged():
    """기존 v0.1 코드가 깨지지 않도록: 원래 필드의 이름, 타입, 순서가 그대로인가"""
    gas = [(f.type, f.name) for f in msgspec.load_all()["GasReading"].fields][:8]
    assert gas == [("std_msgs/Header", "header"), ("float32", "h2s_ppm"), ("float32", "nh3_ppm"),
                   ("float32", "tvoc_ppb"), ("float32", "temperature_c"), ("float32", "humidity_pct"),
                   ("bool", "pump_on"), ("float32", "payload_voltage")]
    odor = [f.name for f in msgspec.load_all()["OdorSample"].fields]
    assert odor[:3] == ["header", "drone", "gas"]


def test_flag_bits_are_shared():
    """GasReading 과 FireStatus 에서 같은 이름의 비트는 같은 값"""
    msgs = msgspec.load_all()
    g = {c.name: c.value for c in msgs["GasReading"].consts}
    f = {c.name: c.value for c in msgs["FireStatus"].consts}
    for name in ("FLAG_TIME_UNSYNCED", "FLAG_SIMULATED"):
        assert g[name] == f[name]
    vals = [c.value for c in msgs["GasReading"].consts if c.name.startswith("FLAG_")]
    assert len(set(vals)) == len(vals) and all(v & (v - 1) == 0 for v in vals)   # 서로 다른 한 비트


@pytest.mark.parametrize("path", EXAMPLES, ids=lambda p: p.stem)
def test_examples_validate(path):
    assert validate_jsonl.errors(load(path)) == []


@pytest.mark.parametrize("mutate,expect", [
    (lambda r: r.update(device_id="FIX-01"), "device_id"),            # 대문자, 하이픈 금지
    (lambda r: r.update(stamp="2026-10-10 14:12:33"), "stamp"),        # Z 없는 현지 시각 금지
    (lambda r: r["gas"].update(nh3_ppm="1.2"), "nh3_ppm"),             # 문자열 숫자 금지
    (lambda r: r.update(sampling_mode=2), "sampling_mode"),            # 정의 안 된 모드
    (lambda r: r.pop("seq"), "seq"),                                   # 필수 필드 누락
    (lambda r: r.update(extra=1), "extra"),                            # 정의 안 된 필드
    (lambda r: r.update(version="0.1"), "version"),
])
def test_validator_rejects_bad_records(mutate, expect):
    rec = copy.deepcopy(load(ROOT / "interfaces" / "examples" / "station_sample.json"))
    mutate(rec)
    errs = validate_jsonl.errors(rec)
    assert errs and any(expect in e for e in errs), errs


@pytest.mark.parametrize("path", EXAMPLES, ids=lambda p: p.stem)
def test_json_ros_round_trip(path):
    classes = build_classes()
    rec = load(path)
    msg = odor_json.fill_msg(classes[CLASS_OF[rec["type"]]](), rec)
    back = odor_json.msg_to_dict(msg, rec["type"])
    assert back == rec


def test_float64_latitude_is_not_truncated():
    classes = build_classes()
    rec = load(ROOT / "interfaces" / "examples" / "station_sample.json")
    msg = odor_json.fill_msg(classes["StationSample"](), rec)
    assert msg.latitude == pytest.approx(35.8242) and isinstance(msg.latitude, float)


def test_null_becomes_nan_and_back():
    classes = build_classes()
    rec = load(ROOT / "interfaces" / "examples" / "gas_reading.json")
    msg = odor_json.fill_msg(classes["GasReading"](), rec)
    assert math.isnan(msg.voc_index) and math.isnan(msg.payload_voltage)
    assert odor_json.msg_to_dict(msg, "gas_reading")["voc_index"] is None


def test_iso_utc_format():
    assert odor_json.iso_utc(1791609153.12) == "2026-10-10T05:12:33.120Z"
    assert odor_json.parse_iso("2026-10-10T05:12:33.120Z") == pytest.approx(1791609153.12)
