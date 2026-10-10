"""gas_reader: 센서 4종 -> GasReading v0.2 값, 품질 비트, 장애 처리"""
import math
import sys
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
MSG_DIR = PKG.parent / "odor_msgs" / "msg"
TOOLS = PKG.parents[2] / "tools"            # moving-ver/tools (규격 파서)

from odor_drone import gas_reader as gr  # noqa: E402
from odor_drone.drivers.dfrobot_gas import GravityGas  # noqa: E402
from odor_drone.drivers.fakes import FakeBME688, FakeGravity, FakeMuxBus, FakeSGP30  # noqa: E402
from odor_drone.drivers.sgp30 import SGP30  # noqa: E402
from odor_drone.drivers.tca9548a import TCA9548A  # noqa: E402
from odor_drone.fake_values import fake_gas  # noqa: E402


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def rig(clock, temp_comp=(), synced=True, warmup_s=180.0):
    bus = FakeMuxBus()
    dev = dict(nh3=bus.attach_ch(0, 0x74, FakeGravity("NH3", 2.0)),
               h2s=bus.attach_ch(1, 0x74, FakeGravity("H2S", 0.30)),
               sgp=bus.attach_ch(2, 0x58, FakeSGP30(tvoc=150, eco2=520)),
               bme=FakeBME688(t=25.0, rh=50.0, p=1008.0, gas_ohm=48000.0))
    tca = TCA9548A(bus)
    sl = lambda _: None  # noqa: E731
    r = gr.GasReader(
        nh3=GravityGas(tca.channel(0), 0x74, "NH3", sleep=sl),
        h2s=GravityGas(tca.channel(1), 0x74, "H2S", sleep=sl),
        sgp30=SGP30(tca.channel(2), sleep=sl, clock=clock),
        bme688=dev["bme"], mux=tca,
        cfg=gr.ReaderConfig(warmup_s=warmup_s, temp_comp=temp_comp),
        clock=clock, clock_synced=lambda: synced)
    return r, dev


def msg_consts(name):
    sys.path.insert(0, str(TOOLS))
    import msgspec
    return msgspec.load_all(MSG_DIR)[name]


def test_constants_match_msg_files():
    gas = msg_consts("GasReading")
    for c in gas.consts:
        assert getattr(gr, c.name) == c.value, c.name
    health = {c.name: c.value for c in msg_consts("DeviceHealth").consts}
    assert [gr.STATE_BOOT, gr.STATE_WARMUP, gr.STATE_OK, gr.STATE_DEGRADED, gr.STATE_ERROR] == \
        [health[k] for k in ("STATE_BOOT", "STATE_WARMUP", "STATE_OK", "STATE_DEGRADED", "STATE_ERROR")]


def test_reader_fills_exactly_the_gasreading_fields():
    clock = Clock()
    r, _ = rig(clock)
    out = r.read()
    names = [f.name for f in msg_consts("GasReading").fields if f.name != "header"]
    assert list(out) == names
    assert fake_gas(0.0).keys() == out.keys()       # 가짜 노드도 같은 필드


def test_normal_reading_after_warmup():
    clock = Clock()
    r, _ = rig(clock)
    assert r.state == gr.STATE_BOOT
    first = r.read()
    assert first["flags"] & gr.FLAG_WARMUP and first["flags"] & gr.FLAG_VOC_INVALID   # SGP30 15 초
    assert math.isnan(first["tvoc_ppb"]) and r.state == gr.STATE_WARMUP
    clock.t += 200
    out = r.read()
    assert out["flags"] == 0, bin(out["flags"])
    assert out["nh3_ppm"] == pytest.approx(2.0) and out["nh3_raw"] == pytest.approx(2.0)
    assert out["h2s_ppm"] == pytest.approx(0.30)
    assert out["tvoc_ppb"] == 150 and out["eco2_ppm"] == 520
    assert out["temperature_c"] == 25.0 and out["pressure_hpa"] == 1008.0 and out["gas_resistance_ohm"] == 48000.0
    assert out["pump_on"] is False
    for k in ("voc_index", "nox_index", "pm2_5_ugm3", "payload_voltage"):   # 이동형에 없는 센서
        assert math.isnan(out[k])
    assert r.sensor_ok_mask == 0b11111 and r.state == gr.STATE_OK


def test_sgp30_gets_humidity_compensation():
    clock = Clock()
    r, dev = rig(clock)
    r.read()
    assert dev["sgp"].humidity_ticks == pytest.approx(11.48 * 256, abs=2)   # 25 ℃, 50 % -> 11.48 g/m³


def test_one_glitch_is_nan_three_is_lost_then_recovers():
    clock = Clock()
    r, dev = rig(clock, warmup_s=0)
    r.read()
    dev["nh3"].fail = True
    out = r.read()
    assert math.isnan(out["nh3_ppm"]) and out["flags"] & gr.FLAG_NH3_INVALID
    assert r.slots["NH3"].ok and r.error_count == 1         # 한 번은 그냥 넘어간다
    r.read()
    r.read()
    assert not r.slots["NH3"].ok and r.sensor_ok_mask & gr.OK_NH3 == 0
    assert r.state == gr.STATE_DEGRADED and "NH3" in r.last_error
    dev["nh3"].fail = False
    r.read()
    assert not r.slots["NH3"].ok                             # 30 초 전에는 다시 시도하지 않는다
    clock.t += 31
    out = r.read()
    assert r.slots["NH3"].ok and out["nh3_ppm"] == pytest.approx(2.0)


def test_corrupt_frames_count_as_errors():
    clock = Clock()
    r, dev = rig(clock, warmup_s=0)
    dev["h2s"].corrupt = True
    out = r.read()
    assert out["flags"] & gr.FLAG_H2S_INVALID and math.isnan(out["h2s_ppm"])


def test_missing_sensors_at_start():
    clock = Clock()
    r, dev = rig(clock, warmup_s=0)
    dev["bme"].fail = True
    dev["sgp"].fail = True
    r.begin()
    out = r.read()
    assert out["flags"] & gr.FLAG_TH_INVALID and out["flags"] & gr.FLAG_VOC_INVALID
    assert math.isnan(out["temperature_c"]) and not math.isnan(out["nh3_ppm"])
    assert r.state == gr.STATE_DEGRADED


def test_all_gas_sensors_down_is_error():
    clock = Clock()
    r, dev = rig(clock, warmup_s=0)
    for k in ("nh3", "h2s", "sgp"):
        dev[k].fail = True
    r.begin()
    assert r.state == gr.STATE_ERROR


def test_temp_comp_only_where_asked():
    clock = Clock()
    r, _ = rig(clock, temp_comp=("NH3",), warmup_s=0)
    out = r.read()
    assert out["flags"] & gr.FLAG_TEMP_COMP
    assert out["nh3_raw"] == pytest.approx(2.0)
    assert out["nh3_ppm"] == pytest.approx(2.0 / 1.205 + 0.5, abs=0.01)   # 보드 25 ℃
    assert out["h2s_ppm"] == pytest.approx(0.30)                           # H2S 는 그대로


def test_time_unsynced_flag():
    clock = Clock()
    r, _ = rig(clock, synced=False, warmup_s=0)
    assert r.read()["flags"] & gr.FLAG_TIME_UNSYNCED


def test_fake_values_are_marked_and_peak_in_plume():
    assert fake_gas(5.0)["flags"] == gr.FLAG_SIMULATED
    assert fake_gas(5.0)["nh3_ppm"] > fake_gas(25.0)["nh3_ppm"] + 3
