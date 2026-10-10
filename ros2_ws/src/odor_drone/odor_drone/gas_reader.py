"""
gas_reader.py - 이동형(mob_01) 센서 4종을 한 번 읽어 GasReading v0.2 값과 품질 비트를 만든다.

ROS 와 무관하다. gas_i2c_node 가 1 초마다 read() 를 불러 /odor/gas 로 발행한다.
테스트는 가짜 센서를 넣어 PC 에서 돌린다 (test/test_gas_reader.py).

센서 (기본 배치, TCA9548A 채널)
  ch0  Gravity NH3  (0x74)  -> nh3_ppm, nh3_raw
  ch1  Gravity H2S  (0x74)  -> h2s_ppm, h2s_raw
  ch2  SGP30        (0x58)  -> tvoc_ppb, eco2_ppm
  ch3  BME688       (0x76)  -> temperature_c, humidity_pct, pressure_hpa, gas_resistance_ohm

이동형에 없는 값: voc_index, nox_index (SGP41), pm*, payload_voltage -> NaN, pump_on -> false.
없는 센서 때문에는 INVALID 비트를 세우지 않는다. INVALID 는 "있어야 할 값이 없다" 는 뜻.

오류 처리
  - 한 번 읽기 실패: 그 값만 NaN + INVALID 비트, error_count 증가
  - 3 번 연속 실패: 그 센서를 끊긴 것으로 보고 retry_s(30 초)마다 다시 begin()
    (드론 진동으로 커넥터가 잠깐 빠졌다 붙는 경우를 위해)
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

from .drivers.dfrobot_gas import vendor_temp_correct
from .drivers.sgp30 import absolute_humidity_g_m3

NAN = float("nan")

# odor_msgs/GasReading.msg 상수와 같은 값 (test_gas_reader 가 .msg 와 대조한다)
FLAG_WARMUP = 1
FLAG_NH3_INVALID = 2
FLAG_H2S_INVALID = 4
FLAG_VOC_INVALID = 8
FLAG_TH_INVALID = 16
FLAG_PM_INVALID = 32
FLAG_TIME_UNSYNCED = 64
FLAG_ZERO_CAL = 128
FLAG_PURGE = 256
FLAG_PUMP_ON = 512
FLAG_TEMP_COMP = 1024
FLAG_SIMULATED = 2048

# DeviceHealth.sensor_ok_mask 비트 (이동형 mob_01). DATA_SPEC.md 4.6
OK_NH3 = 1
OK_H2S = 2
OK_SGP30 = 4
OK_BME688 = 8
OK_TCA9548A = 16

# DeviceHealth.state
STATE_BOOT, STATE_WARMUP, STATE_OK, STATE_DEGRADED, STATE_ERROR = range(5)

FAILS_BEFORE_LOST = 3
SENSOR_ERRORS = (OSError, IOError, ValueError)   # CrcError, GasProtocolError 는 IOError 의 하위


@dataclass
class ReaderConfig:
    warmup_s: float = 180.0            # 전기화학 센서 예열. 이 시간 동안 FLAG_WARMUP
    temp_comp: tuple[str, ...] = ()    # 제조사 온도 보정을 적용할 가스 ("NH3", "H2S"). 기본은 안 함
    temp_every: int = 10               # Gravity 보드 온도는 10 회에 한 번만 읽는다 (천천히 변함)
    retry_s: float = 30.0              # 끊긴 센서 다시 붙여 보기 간격
    humidity_update_s: float = 60.0    # SGP30 습도 보정값 갱신 주기 (크게 바뀌면 바로)


@dataclass
class Slot:
    name: str
    bit: int
    sensor: object
    ok: bool = False
    fails: int = 0
    next_retry: float = 0.0
    last_error: str = ""
    info: str = ""


@dataclass
class _Cache:
    board_temp: dict = field(default_factory=dict)   # 가스 이름 -> 보드 온도 ℃
    reads: int = 0
    ah_sent: float = -1.0
    ah_sent_at: float = -1e9


class GasReader:
    def __init__(self, nh3=None, h2s=None, sgp30=None, bme688=None, mux=None,
                 cfg: ReaderConfig | None = None, clock=time.monotonic, clock_synced=lambda: True,
                 log=lambda msg: None):
        self.cfg = cfg or ReaderConfig()
        self.clock = clock
        self.clock_synced = clock_synced
        self.log = log
        self.mux = mux
        self.slots = {n: Slot(n, b, s) for n, b, s in (("NH3", OK_NH3, nh3), ("H2S", OK_H2S, h2s),
                                                      ("SGP30", OK_SGP30, sgp30), ("BME688", OK_BME688, bme688))
                      if s is not None}
        self.started_at: float | None = None
        self.error_count = 0
        self.last_error = ""
        self._c = _Cache()

    # ------------------------------------------------------------ 상태
    @property
    def sensor_ok_mask(self) -> int:
        m = sum(s.bit for s in self.slots.values() if s.ok)
        if self.mux is not None and self.mux.probe():
            m |= OK_TCA9548A
        return m

    @property
    def warming_up(self) -> bool:
        return self.started_at is None or self.clock() - self.started_at < self.cfg.warmup_s

    @property
    def state(self) -> int:
        if self.started_at is None:
            return STATE_BOOT
        gas_ok = [s.ok for n, s in self.slots.items() if n in ("NH3", "H2S", "SGP30")]
        if not any(gas_ok):
            return STATE_ERROR
        if self.warming_up:
            return STATE_WARMUP
        return STATE_OK if all(s.ok for s in self.slots.values()) else STATE_DEGRADED

    def _fail(self, slot: Slot, err: Exception, during: str) -> None:
        self.error_count += 1
        slot.fails += 1
        slot.last_error = f"{slot.name} {during}: {err}"
        self.last_error = slot.last_error
        if slot.ok and slot.fails >= FAILS_BEFORE_LOST:
            slot.ok = False
            slot.next_retry = self.clock() + self.cfg.retry_s
            self.log(f"{slot.name} 끊김으로 판단 ({slot.fails} 회 연속 실패). {self.cfg.retry_s:.0f} 초마다 다시 연결 시도")
        elif not slot.ok:
            slot.next_retry = self.clock() + self.cfg.retry_s

    # ------------------------------------------------------------ 시작
    def _begin(self, slot: Slot) -> bool:
        try:
            info = slot.sensor.begin()
        except SENSOR_ERRORS as e:
            self._fail(slot, e, "연결")
            return False
        slot.ok, slot.fails = True, 0
        slot.info = info if isinstance(info, str) else f"{slot.name} 0x{info:04x}"
        if slot.name == "SGP30":
            self._c.ah_sent = -1.0          # 다시 붙었으면 습도 보정도 다시 보낸다
        self.log(f"{slot.name} 연결됨 ({info})")
        return True

    def begin(self) -> int:
        """센서를 모두 시작해 보고 정상인 센서 비트를 돌려준다. 하나도 없어도 예외는 내지 않는다."""
        for slot in self.slots.values():
            self._begin(slot)
        self.started_at = self.clock()
        return self.sensor_ok_mask

    def _retry_lost(self) -> None:
        now = self.clock()
        for slot in self.slots.values():
            if not slot.ok and now >= slot.next_retry:
                self._begin(slot)

    # ------------------------------------------------------------ 읽기
    def _read(self, name: str, fn):
        """fn(sensor) 를 불러 값을 돌려준다. 센서가 없거나, 끊겼거나, 실패하면 None"""
        slot = self.slots.get(name)
        if slot is None or not slot.ok:
            return None
        try:
            val = fn(slot.sensor)
        except SENSOR_ERRORS as e:
            self._fail(slot, e, "읽기")
            return None
        slot.fails = 0
        return val

    def read(self) -> dict:
        """GasReading v0.2 의 header 를 뺀 필드 전부를 dict 로"""
        if self.started_at is None:
            self.begin()
        self._retry_lost()
        c = self._c
        c.reads += 1
        now = self.clock()
        flags = 0
        out = dict(h2s_ppm=NAN, nh3_ppm=NAN, tvoc_ppb=NAN, temperature_c=NAN, humidity_pct=NAN,
                   pump_on=False, payload_voltage=NAN,
                   eco2_ppm=NAN, voc_index=NAN, nox_index=NAN, pressure_hpa=NAN, gas_resistance_ohm=NAN,
                   pm1_0_ugm3=NAN, pm2_5_ugm3=NAN, pm10_ugm3=NAN, nh3_raw=NAN, h2s_raw=NAN)

        # 1) 온습도, 기압 (SGP30 습도 보정에 먼저 필요)
        if "BME688" in self.slots:
            b = self._read("BME688", lambda s: s.read())
            if b is None:
                flags |= FLAG_TH_INVALID
            else:
                out.update(temperature_c=b.temperature_c, humidity_pct=b.humidity_pct,
                           pressure_hpa=b.pressure_hpa, gas_resistance_ohm=b.gas_resistance_ohm)

        # 2) TVOC, eCO2
        if "SGP30" in self.slots:
            t, rh = out["temperature_c"], out["humidity_pct"]
            if not (math.isnan(t) or math.isnan(rh)):
                ah = absolute_humidity_g_m3(t, rh)
                if abs(ah - c.ah_sent) > 0.5 or now - c.ah_sent_at > self.cfg.humidity_update_s:
                    if self._read("SGP30", lambda s: s.set_humidity(ah) or True):
                        c.ah_sent, c.ah_sent_at = ah, now
            m = self._read("SGP30", lambda s: (s.measure(), s.warming_up))
            if m is None or m[1]:
                flags |= FLAG_VOC_INVALID          # 실패, 또는 켠 뒤 15 초(값이 400, 0 으로 고정)
            else:
                (eco2, tvoc), _ = m
                out.update(eco2_ppm=float(eco2), tvoc_ppb=float(tvoc))

        # 3) 전기화학 NH3, H2S
        compensated = False
        for gas, key, bad in (("NH3", "nh3", FLAG_NH3_INVALID), ("H2S", "h2s", FLAG_H2S_INVALID)):
            if gas not in self.slots:
                continue
            v = self._read(gas, lambda s: s.read())
            if v is None:
                flags |= bad
                continue
            out[f"{key}_raw"] = v.ppm
            out[f"{key}_ppm"] = v.ppm
            if gas in self.cfg.temp_comp:
                if gas not in c.board_temp or c.reads % self.cfg.temp_every == 1:
                    bt = self._read(gas, lambda s: s.read_temp_c())
                    if bt is not None:
                        c.board_temp[gas] = bt
                fixed = vendor_temp_correct(gas, v.ppm, c.board_temp.get(gas, NAN))
                if fixed is not None:
                    out[f"{key}_ppm"] = fixed
                    compensated = True

        if compensated:
            flags |= FLAG_TEMP_COMP
        if self.warming_up:
            flags |= FLAG_WARMUP
        if not self.clock_synced():
            flags |= FLAG_TIME_UNSYNCED
        out["flags"] = flags
        return out
