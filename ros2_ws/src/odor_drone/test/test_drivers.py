"""센서 드라이버 단위 시험 (하드웨어, ROS 없이). 실행: python3 -m pytest -q ros2_ws/src/odor_drone/test"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from odor_drone.drivers import dfrobot_gas as dg  # noqa: E402
from odor_drone.drivers.fakes import FakeGravity, FakeMuxBus, FakeSGP30  # noqa: E402
from odor_drone.drivers.i2c_bus import FakeBus  # noqa: E402
from odor_drone.drivers.sensirion import CrcError, crc8, pack_words, unpack_words  # noqa: E402
from odor_drone.drivers.sgp30 import SGP30, absolute_humidity_g_m3, ah_ticks  # noqa: E402
from odor_drone.drivers.tca9548a import TCA9548A  # noqa: E402


def no_sleep(_):
    pass


# ------------------------------------------------------------------ Gravity
def test_dfrobot_frames_match_vendor_examples():
    # 제조사 설명서의 명령 예: 농도 읽기 FF 01 86 00 00 00 00 00 79, passive 전환 FF 01 78 04 00 00 00 00 83
    assert dg.pack(0x86).hex(" ") == "ff 01 86 00 00 00 00 00 79"
    assert dg.pack(0x78, 0x04).hex(" ") == "ff 01 78 04 00 00 00 00 83"


def test_dfrobot_checksum_skips_last_data_byte():
    """제조사 FucCheckSum(buf, 8) 은 1~6 번째만 더한다. 7 번째가 바뀌어도 체크섬은 같다"""
    a = bytes([0xFF, 0x88, 0x00, 0x7B, 0x02, 0x01, 0x02, 0x00, 0])
    b = bytes([0xFF, 0x88, 0x00, 0x7B, 0x02, 0x01, 0x02, 0xAB, 0])
    assert dg.checksum(a) == dg.checksum(b)


def test_parse_concentration_and_thermistor():
    resp = bytearray([0xFF, 0x86, 0x00, 0x7B, 0x02, 0x01, 0, 0, 0])
    resp[8] = dg.checksum(resp)
    ppm, gas = dg.parse_concentration(dg.check(bytes(resp), 0x86))
    assert gas == "NH3" and ppm == pytest.approx(12.3)
    assert dg.thermistor_c(512) == pytest.approx(25.0, abs=0.01)     # 중간 전압 = 10 kΩ = 25 ℃
    assert math.isnan(dg.thermistor_c(0))


def test_bad_checksum_raises():
    resp = bytearray([0xFF, 0x86, 0, 10, 2, 0, 0, 0, 0])
    resp[8] = dg.checksum(resp) ^ 1
    with pytest.raises(dg.GasProtocolError):
        dg.check(bytes(resp))


def test_gravity_through_mux_two_same_address():
    bus = FakeMuxBus()
    bus.attach_ch(0, 0x74, FakeGravity("NH3", 3.25))
    bus.attach_ch(1, 0x74, FakeGravity("H2S", 0.42))
    tca = TCA9548A(bus)
    nh3 = dg.GravityGas(tca.channel(0), 0x74, "NH3", sleep=no_sleep)
    h2s = dg.GravityGas(tca.channel(1), 0x74, "H2S", sleep=no_sleep)
    assert nh3.begin() == "NH3" and h2s.begin() == "H2S"
    assert nh3.read().ppm == pytest.approx(3.25)
    assert h2s.read().ppm == pytest.approx(0.42)
    assert nh3.read_temp_c() == pytest.approx(25.0, abs=0.01)
    # 멀티플렉서는 채널이 바뀔 때만 다시 쓴다: begin 2 번(ch0, ch1) + 읽기 3 번(ch0, ch1, ch0)
    writes = [x[2] for x in bus.log if x[0] == "w" and x[1] == 0x70]
    assert writes == [b"\x01", b"\x02", b"\x01", b"\x02", b"\x01"]


def test_swapped_sensor_is_reported():
    bus = FakeBus()
    bus.attach(0x74, FakeGravity("H2S", 1.0))
    with pytest.raises(dg.GasProtocolError, match="NH3 자리에 H2S"):
        dg.GravityGas(bus, 0x74, "NH3", sleep=no_sleep).begin()


def test_missing_device_is_oserror():
    with pytest.raises(OSError):
        dg.GravityGas(FakeBus(), 0x74, "NH3", sleep=no_sleep).begin()


def test_vendor_temp_correction():
    # NH3 20~40 ℃: c / (0.005 t + 1.08) - (-0.1 t + 2)
    assert dg.vendor_temp_correct("NH3", 10.0, 25.0) == pytest.approx(10.0 / 1.205 + 0.5)
    assert dg.vendor_temp_correct("NH3", 10.0, 45.0) is None        # 식의 범위 밖
    assert dg.vendor_temp_correct("H2S", 1.0, 10.0) == pytest.approx(1.0 / 0.97)
    # 제조사 H2S 식은 20 ℃ 를 넘으면 25 ℃ 에서 13 배가 된다 -> 기본으로 쓰지 않는 이유
    assert dg.vendor_temp_correct("H2S", 1.0, 25.0) == pytest.approx(1.0 / 0.075)
    assert dg.vendor_temp_correct("NH3", 0.1, 30.0) == pytest.approx(0.1 / 1.23 + 1.0)
    assert dg.vendor_temp_correct("CO", 1.0, 25.0) is None
    assert dg.vendor_temp_correct("NH3", 1.0, float("nan")) is None


# ------------------------------------------------------------------ Sensirion, SGP30
def test_sensirion_crc():
    assert crc8(b"\xBE\xEF") == 0x92                   # 데이터시트 예시
    assert unpack_words(pack_words(0x1234, 0xBEEF)) == [0x1234, 0xBEEF]
    bad = bytearray(pack_words(0xBEEF))
    bad[2] ^= 1
    with pytest.raises(CrcError):
        unpack_words(bytes(bad))


def test_absolute_humidity():
    assert absolute_humidity_g_m3(25.0, 50.0) == pytest.approx(11.5, abs=0.1)
    assert ah_ticks(11.5) == 2944
    assert ah_ticks(500) == 0xFFFF and ah_ticks(-1) == 0


def test_sgp30_sequence():
    now = [100.0]
    bus = FakeBus()
    dev = bus.attach(0x58, FakeSGP30(tvoc=250, eco2=612))
    s = SGP30(bus, sleep=no_sleep, clock=lambda: now[0])
    assert s.begin() == 0x0022 and dev.inited
    assert s.warming_up
    assert s.measure() == (612, 250)
    now[0] += 15.1
    assert not s.warming_up
    s.set_humidity(11.5)
    assert dev.humidity_ticks == 2944
    s.set_baseline(eco2=0x1111, tvoc=0x2222)            # 보낼 때는 TVOC 먼저
    assert dev.baseline == (0x1111, 0x2222)
    assert s.get_baseline() == (0x1111, 0x2222)
    assert s.serial_id() == 0x01234567


def test_sgp30_rejects_other_chip():
    bus = FakeBus()
    bus.attach(0x58, FakeSGP30(feature_set=0x1022))
    with pytest.raises(IOError):
        SGP30(bus, sleep=no_sleep).begin()


def test_mux_forgets_channel_after_error():
    bus = FakeMuxBus()
    g = bus.attach_ch(2, 0x74, FakeGravity("NH3", 1.0))
    tca = TCA9548A(bus)
    sensor = dg.GravityGas(tca.channel(2), 0x74, "NH3", sleep=no_sleep)
    sensor.begin()
    g.fail = True
    with pytest.raises(OSError):
        sensor.read()
    assert tca._current is None           # 다음 전송에서 채널을 다시 쓴다
    g.fail = False
    assert sensor.read().ppm == pytest.approx(1.0)
