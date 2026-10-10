"""
fakes.py - 하드웨어 없이 드라이버를 시험하기 위한 가짜 I2C 장치들

  FakeMuxBus     TCA9548A(0x70) 가 달린 버스. 채널마다 장치를 붙인다 (같은 주소 0x74 두 개 가능)
  FakeGravity    Gravity NH3/H2S. 9 B 명령 프레임에 제조사 형식으로 응답
  FakeSGP30      SGP30. 명령 2 B (+ 워드) 에 Sensirion 형식으로 응답
  FakeBME688     BME688 은 레지스터 흉내가 길어서 드라이버(BME688 클래스) 자체를 흉내 낸다

테스트(test/)와 gas_i2c_node 의 simulate_bus 모드(개발용)가 쓴다.
값은 함수로 넣는다: FakeGravity("NH3", ppm=lambda: 1.2)
"""
from __future__ import annotations

import math

from .bme688 import BmeValue
from .dfrobot_gas import GAS_TYPES, checksum
from .i2c_bus import FakeBus, FakeDevice
from .sensirion import pack_words, unpack_words

GAS_CODE = {v: k for k, v in GAS_TYPES.items()}


def _const(v):
    return lambda: v


class FakeMuxBus(FakeBus):
    """0x70 에 TCA9548A 가 있고, 그 뒤 채널 0~7 에 장치가 있는 버스"""

    def __init__(self, tca_addr: int = 0x70):
        super().__init__()
        self.tca_addr = tca_addr
        self.mask = 0
        self.behind: dict[int, dict[int, FakeDevice]] = {ch: {} for ch in range(8)}

    def attach_ch(self, ch: int, addr: int, dev: FakeDevice) -> FakeDevice:
        self.behind[ch][addr] = dev
        return dev

    def _dev(self, addr):
        if addr == self.tca_addr:
            return _TcaReg(self)
        if addr in self.devices:
            return self.devices[addr]
        found = [self.behind[ch][addr] for ch in range(8) if self.mask >> ch & 1 and addr in self.behind[ch]]
        if len(found) > 1:
            raise OSError(5, f"0x{addr:02x} 주소 충돌 (켜진 채널 여러 개)")
        if not found:
            raise OSError(121, f"Remote I/O error (0x{addr:02x} 응답 없음, 채널 마스크 0b{self.mask:08b})")
        return found[0]


class _TcaReg(FakeDevice):
    def __init__(self, bus: FakeMuxBus):
        self.bus = bus

    def on_write(self, data):
        self.bus.mask = data[-1]

    def on_read(self, n):
        return bytes([self.bus.mask] * n)


class FakeGravity(FakeDevice):
    def __init__(self, gas: str = "NH3", ppm=0.0, temp_adc: int = 512, decimals: int = 2):
        self.gas = gas
        self.ppm = ppm if callable(ppm) else _const(ppm)
        self.temp_adc = temp_adc          # 512 -> 25.0 ℃
        self.decimals = decimals
        self.mode = 0x03
        self.cmd = None
        self.fail = False                 # True 면 응답 없음 (커넥터 빠짐 흉내)
        self.corrupt = False              # True 면 체크섬 틀린 응답

    def on_write(self, data):
        if self.fail:
            raise OSError(121, "Remote I/O error")
        if len(data) == 10 and data[1] == 0xFF:      # 레지스터 0x00 + 9 B 명령
            frame = data[1:]
            if checksum(frame) != frame[8]:
                raise AssertionError(f"드라이버가 보낸 체크섬이 틀림: {frame.hex(' ')}")
            self.cmd = frame[2]
            if self.cmd == 0x78:
                self.mode = frame[3]

    def on_read(self, n):
        if self.fail:
            raise OSError(121, "Remote I/O error")
        d = [0] * 6
        if self.cmd == 0x78:
            d[0] = 1
        elif self.cmd == 0x86:
            raw = int(round(max(0.0, self.ppm()) * 10 ** self.decimals))
            d[:4] = [raw >> 8 & 0xFF, raw & 0xFF, GAS_CODE[self.gas], self.decimals]
        elif self.cmd == 0x87:
            d[:2] = [self.temp_adc >> 8, self.temp_adc & 0xFF]
        elif self.cmd == 0x91:
            d[:2] = [0x01, 0x00]
        frame = bytearray([0xFF, self.cmd or 0] + d + [0])
        frame[8] = checksum(frame) ^ (0xFF if self.corrupt else 0)
        return bytes(frame[:n])


class FakeSGP30(FakeDevice):
    def __init__(self, tvoc=0.0, eco2=None, feature_set: int = 0x0022):
        self.tvoc = tvoc if callable(tvoc) else _const(tvoc)
        self.eco2 = eco2 if callable(eco2) else (_const(eco2) if eco2 is not None else
                                                 (lambda: 400 + 0.8 * self.tvoc()))
        self.feature_set = feature_set
        self.cmd = None
        self.humidity_ticks = None
        self.baseline = (0x8973, 0x8AAE)   # (eCO2, TVOC)
        self.inited = False
        self.fail = False

    def on_write(self, data):
        if self.fail:
            raise OSError(121, "Remote I/O error")
        self.cmd = data[0] << 8 | data[1]
        words = unpack_words(data[2:]) if len(data) > 2 else []
        if self.cmd == 0x2003:
            self.inited = True
        elif self.cmd == 0x2061:
            self.humidity_ticks = words[0]
        elif self.cmd == 0x201E:
            tvoc_b, eco2_b = words
            self.baseline = (eco2_b, tvoc_b)

    def on_read(self, n):
        if self.fail:
            raise OSError(121, "Remote I/O error")
        if self.cmd == 0x202F:
            words = [self.feature_set]
        elif self.cmd == 0x2008:
            words = [int(round(self.eco2())), int(round(self.tvoc()))]
        elif self.cmd == 0x2015:
            words = list(self.baseline)
        elif self.cmd == 0x3682:
            words = [0x0000, 0x0123, 0x4567]
        else:
            words = [0] * (n // 3)
        return pack_words(*words)[:n]


class FakeBME688:
    """BME688 드라이버와 같은 begin(), read() 를 가진 가짜"""

    def __init__(self, t=25.0, rh=50.0, p=1008.0, gas_ohm=50000.0):
        self.values = {k: (v if callable(v) else _const(v)) for k, v in
                       dict(t=t, rh=rh, p=p, gas_ohm=gas_ohm).items()}
        self.fail = False
        self.started = False

    def begin(self):
        if self.fail:
            raise OSError(121, "BME688 없음")
        self.started = True
        return "BME688"

    def read(self):
        if self.fail or not self.started:
            raise OSError(121, "BME688 응답 없음")
        g = self.values["gas_ohm"]()
        return BmeValue(self.values["t"](), self.values["rh"](), self.values["p"](),
                        g if not math.isnan(g) else float("nan"), True)
