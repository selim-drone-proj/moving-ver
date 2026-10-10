"""
sgp30.py - Sensirion SGP30 (TVOC, eCO2) I2C 드라이버. 주소 0x58 고정.

명령 (2 B, 응답은 16비트 + CRC 묶음. sensirion.py)
  0x2003 iaq_init           켠 뒤 한 번. 이후 15 초 동안은 eCO2 400, TVOC 0 만 나온다
  0x2008 measure_iaq        12 ms 뒤 6 B: eCO2 ppm, TVOC ppb. 1 초마다 불러야 기준선 학습이 맞게 돈다
  0x2015 get_iaq_baseline   10 ms 뒤 6 B: eCO2 기준선, TVOC 기준선
  0x201E set_iaq_baseline   + TVOC 기준선, eCO2 기준선 (읽을 때와 순서가 반대)
  0x2061 set_absolute_humidity + 절대습도(g/m³, 8.8 고정소수점). 0 이면 습도 보정 끔
  0x202F get_feature_set    10 ms 뒤 3 B: 상위 4비트 0 = SGP30
  0x3682 get_serial_id      0.5 ms 뒤 9 B

eCO2 는 TVOC 에서 계산한 추정값이지 실제 CO2 가 아니다 (메시지 설명에도 적어 둠).
"""
from __future__ import annotations

import math
import time

from .i2c_bus import I2CBus
from .sensirion import pack_words, unpack_words

ADDR = 0x58
WARMUP_S = 15.0


def absolute_humidity_g_m3(t_c: float, rh_pct: float) -> float:
    """온도, 상대습도 -> 절대습도 g/m³ (Sensirion 응용 노트 식)"""
    es = 6.112 * math.exp(17.62 * t_c / (243.12 + t_c))       # 포화 수증기압 hPa
    return 216.7 * (rh_pct / 100.0 * es) / (273.15 + t_c)


def ah_ticks(ah_g_m3: float) -> int:
    """절대습도 -> 8.8 고정소수점 (최대 255.996 g/m³)"""
    return max(0, min(0xFFFF, int(round(ah_g_m3 * 256.0))))


class SGP30:
    def __init__(self, bus: I2CBus, addr: int = ADDR, sleep=time.sleep, clock=time.monotonic):
        self.bus = bus
        self.addr = addr
        self.sleep = sleep
        self.clock = clock
        self.started_at: float | None = None
        self.feature_set: int | None = None

    def _cmd(self, cmd: int, words: tuple = (), wait_s: float = 0.0, n_read: int = 0) -> list[int]:
        self.bus.write(self.addr, bytes([cmd >> 8, cmd & 0xFF]) + pack_words(*words))
        if wait_s:
            self.sleep(wait_s)
        return unpack_words(self.bus.read(self.addr, n_read * 3)) if n_read else []

    def begin(self) -> int:
        fs = self._cmd(0x202F, wait_s=0.01, n_read=1)[0]
        if fs >> 12 != 0:
            raise IOError(f"SGP30 가 아님 (feature set 0x{fs:04x})")
        self.feature_set = fs
        self._cmd(0x2003, wait_s=0.01)
        self.started_at = self.clock()
        return fs

    @property
    def warming_up(self) -> bool:
        return self.started_at is None or self.clock() - self.started_at < WARMUP_S

    def measure(self) -> tuple[int, int]:
        """-> (eCO2 ppm, TVOC ppb)"""
        eco2, tvoc = self._cmd(0x2008, wait_s=0.012, n_read=2)
        return eco2, tvoc

    def get_baseline(self) -> tuple[int, int]:
        eco2, tvoc = self._cmd(0x2015, wait_s=0.01, n_read=2)
        return eco2, tvoc

    def set_baseline(self, eco2: int, tvoc: int) -> None:
        self._cmd(0x201E, (tvoc, eco2), wait_s=0.01)

    def set_humidity(self, ah_g_m3: float) -> None:
        self._cmd(0x2061, (ah_ticks(ah_g_m3),), wait_s=0.01)

    def serial_id(self) -> int:
        a, b, c = self._cmd(0x3682, wait_s=0.001, n_read=3)
        return (a << 32) | (b << 16) | c
