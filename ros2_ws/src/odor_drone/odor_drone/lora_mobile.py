"""
lora_mobile.py - 이동형 LoRa 요약 패킷 (LoRa 패킷 v1, type 0x03) 인코더. 표준 라이브러리만 사용.

규격: stable-ver/protocol/LORA_PACKET.md 3-2절
짝:   stable-ver/gateway/odor_gateway/lora_codec.py (지상 게이트웨이 디코더. 둘을 같이 고칠 것)
둘이 같은 바이트를 만드는지는 양쪽 시험의 같은 시험 벡터로 확인한다.

    fields = sample_fields(odor_sample_msg)
    frame = encode_mobile(station=1, seq=0, epoch_s=fields.pop("epoch_s"), fields=fields)
"""
from __future__ import annotations

import math
import struct

SYNC = b"\xA5\x5A"
VERSION = 1
TYPE_MOBILE = 0x03

HEADER = struct.Struct("<BBBHIB")            # version, type, station, seq, time, len
MOBILE = struct.Struct("<iihhHH4Hh4H3BH")    # 39 바이트

U8_NA = 0xFF
U16_NA = 0xFFFF
I16_NA = -32768
I32_NA = -(2**31)

STATUS_ARMED = 1 << 0
STATUS_IN_AIR = 1 << 1
STATUS_FIX_SHIFT = 2                         # bit2~4: gps_fix_type 0~7

# (필드, 형식, LSB). MOBILE 구조의 순서와 같다. 'status' 는 armed, in_air, gps_fix_type 을 묶은 것
FIELDS = [
    ("latitude", "i32", 1e-7), ("longitude", "i32", 1e-7),
    ("altitude_msl", "i16", 0.1), ("altitude_rel", "i16", 0.1),
    ("heading_deg", "u16", 0.01), ("ground_speed_mps", "u16", 0.01),
    ("nh3_ppm", "u16", 0.01), ("h2s_ppm", "u16", 0.001), ("tvoc_ppb", "u16", 1.0), ("eco2_ppm", "u16", 1.0),
    ("temperature_c", "i16", 0.01), ("humidity_pct", "u16", 0.01), ("pressure_hpa", "u16", 0.1),
    ("gas_resistance_ohm", "u16", 100.0), ("flags", "raw16", 1),
    ("status", "raw8", 1), ("gps_satellites", "raw8", 1), ("drone_age_s", "u8", 0.1),
    ("battery_voltage", "u16", 0.01),
]


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _missing(v) -> bool:
    return v is None or (isinstance(v, float) and not math.isfinite(v))


def _round(x: float) -> int:
    """0.5 에서 멀어지는 쪽으로 반올림 (고정형 펌웨어, 게이트웨이와 같은 규칙)"""
    return int(math.floor(x + 0.5)) if x >= 0 else int(math.ceil(x - 0.5))


def _enc(v, kind: str, lsb: float) -> int:
    if kind.startswith("raw"):
        return int(v)
    if kind == "u8":
        return U8_NA if _missing(v) else _round(max(0.0, min(254.0, v / lsb)))
    if kind == "u16":
        return U16_NA if _missing(v) else _round(max(0.0, min(65534.0, v / lsb)))
    if kind == "i16":
        return I16_NA if _missing(v) else _round(max(-32767.0, min(32767.0, v / lsb)))
    if kind == "i32":
        return I32_NA if _missing(v) else _round(max(-(2**31 - 1), min(2**31 - 1, v / lsb)))
    raise ValueError(kind)


def encode_mobile(station: int, seq: int, epoch_s: int, fields: dict) -> bytes:
    """fields: FIELDS 의 이름 (status 대신 armed, in_air, gps_fix_type 을 줘도 된다). 없는 값은 None/NaN"""
    f = dict(fields)
    if "status" not in f:
        fix = max(0, min(7, int(f.get("gps_fix_type") or 0)))
        f["status"] = ((STATUS_ARMED if f.get("armed") else 0) | (STATUS_IN_AIR if f.get("in_air") else 0)
                       | (fix << STATUS_FIX_SHIFT))
    f.setdefault("flags", 0)
    f["gps_satellites"] = max(0, min(255, int(f.get("gps_satellites") or 0)))
    payload = MOBILE.pack(*(_enc(f.get(name), kind, lsb) for name, kind, lsb in FIELDS))
    body = HEADER.pack(VERSION, TYPE_MOBILE, station, seq & 0xFFFF, int(epoch_s) & 0xFFFFFFFF, len(payload)) + payload
    return SYNC + body + struct.pack("<H", crc16_ccitt(body))


def sample_fields(s) -> dict:
    """OdorSample 메시지 -> encode_mobile 용 dict (+ 'epoch_s' 가스 측정 시각 초). ROS 없이도 같은 속성을 가진 객체면 된다"""
    d, g = s.drone, s.gas
    return {
        "epoch_s": int(g.header.stamp.sec),
        "latitude": d.latitude, "longitude": d.longitude,
        "altitude_msl": d.altitude_msl, "altitude_rel": d.altitude_rel,
        "heading_deg": d.heading_deg, "ground_speed_mps": d.ground_speed_mps,
        "armed": d.armed, "in_air": d.in_air, "gps_fix_type": d.gps_fix_type, "gps_satellites": d.gps_satellites,
        "battery_voltage": d.battery_voltage,
        "nh3_ppm": g.nh3_ppm, "h2s_ppm": g.h2s_ppm, "tvoc_ppb": g.tvoc_ppb, "eco2_ppm": g.eco2_ppm,
        "temperature_c": g.temperature_c, "humidity_pct": g.humidity_pct, "pressure_hpa": g.pressure_hpa,
        "gas_resistance_ohm": g.gas_resistance_ohm, "flags": g.flags,
        "drone_age_s": s.drone_age_s,
    }
