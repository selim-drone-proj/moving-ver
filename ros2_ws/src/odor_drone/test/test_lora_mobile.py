"""이동형 LoRa 요약 패킷(type 0x03) 인코더 시험 (ROS 없이). 시험 벡터는 stable-ver/protocol/LORA_PACKET.md 와 같다"""
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from odor_drone import lora_mobile as lm  # noqa: E402

NAN = float("nan")

# LORA_PACKET.md 6절 vector-mobile 과 같은 값
VECTOR_FIELDS = dict(
    latitude=35.8243715, longitude=127.1486021, altitude_msl=60.2, altitude_rel=20.1, heading_deg=87.5,
    ground_speed_mps=3.1, armed=True, in_air=True, gps_fix_type=3, gps_satellites=15, battery_voltage=23.4,
    nh3_ppm=0.83, h2s_ppm=0.021, tvoc_ppb=240.0, eco2_ppm=520.0, temperature_c=23.1, humidity_pct=58.4,
    pressure_hpa=1008.4, gas_resistance_ohm=52300.0, flags=1024, drone_age_s=0.3,
)
VECTOR = ("a5 5a 01 03 01 07 05 41 c9 c9 6a 27 83 5d 5a 15 45 56 c9 4b 5a 02 c9 00 2e 22 36 01 53 00 15 00 "
          "f0 00 08 02 06 09 d0 16 64 27 0b 02 00 04 0f 0f 03 24 09 d2 84")


def test_crc_check_value():
    assert lm.crc16_ccitt(b"123456789") == 0x29B1


def test_sizes():
    assert lm.MOBILE.size == 39
    assert len(lm.encode_mobile(1, 0, 0, {})) == 53


def test_vector_bytes():
    assert lm.encode_mobile(1, 1287, 1791609153, VECTOR_FIELDS).hex(" ") == VECTOR


def test_missing_values_use_na_codes():
    f = {k: NAN for k in VECTOR_FIELDS}
    f.update(armed=False, in_air=False, gps_fix_type=0, gps_satellites=0, flags=72)
    v = lm.MOBILE.unpack(lm.encode_mobile(2, 0, 0, f)[12:-2])
    assert v[0] == v[1] == lm.I32_NA
    assert v[2] == v[3] == v[10] == lm.I16_NA
    assert v[4] == v[5] == v[6] == v[13] == v[18] == lm.U16_NA
    assert v[14] == 72 and v[15] == 0 and v[17] == lm.U8_NA


def test_clamping_and_negative_values():
    v = lm.MOBILE.unpack(lm.encode_mobile(1, 0, 0, dict(
        latitude=-33.8688197, altitude_rel=-5.25, temperature_c=-10.005, nh3_ppm=-1.0, h2s_ppm=99.0,
        gas_resistance_ohm=9e6, drone_age_s=40.0, gps_fix_type=9))[12:-2])
    assert v[0] == -338688197
    assert v[3] == -53                # -5.25 m / 0.1 -> -52.5 -> 0.5 에서 먼 쪽
    assert v[10] == -1001
    assert v[6] == 0 and v[7] == 65534 and v[13] == 65534 and v[17] == 254
    assert (v[15] >> lm.STATUS_FIX_SHIFT) & 7 == 7


def test_seq_wraps():
    assert lm.encode_mobile(1, 65536 + 5, 0, {})[5:7] == bytes([5, 0])


def test_sample_fields_from_message_like_object():
    stamp = NS(sec=1791609153, nanosec=120_000_000)
    drone = NS(latitude=35.0, longitude=127.0, altitude_msl=NAN, altitude_rel=10.0, heading_deg=90.0,
               ground_speed_mps=2.0, armed=True, in_air=False, gps_fix_type=3, gps_satellites=12,
               battery_voltage=22.8)
    gas = NS(header=NS(stamp=stamp), nh3_ppm=1.0, h2s_ppm=0.01, tvoc_ppb=NAN, eco2_ppm=NAN,
             temperature_c=20.0, humidity_pct=50.0, pressure_hpa=1000.0, gas_resistance_ohm=40000.0, flags=8)
    f = lm.sample_fields(NS(drone=drone, gas=gas, drone_age_s=0.05))
    assert f["epoch_s"] == 1791609153
    assert math.isnan(f["tvoc_ppb"]) and f["flags"] == 8
    frame = lm.encode_mobile(1, 0, f.pop("epoch_s"), f)
    assert len(frame) == 53
