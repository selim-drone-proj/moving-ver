"""
bme688.py - Bosch BME688 (온도, 습도, 기압, 가스 저항) 래퍼

보정 계산은 pimoroni bme680 라이브러리(MIT, 2.0 이상은 BME688 도 처리)에 맡긴다.
    pip install "bme680>=2.0"
주소: 0x76 (SDO=GND) 또는 0x77 (SDO=VCC). 보드마다 기본이 달라서 begin() 이 둘 다 시도한다.
I2C 는 우리 버스 객체(i2c_bus.I2CBus)를 그대로 넘기므로 TCA9548A 채널 위에서도 동작한다.

가스 저항은 히터가 목표 온도에 도달했을 때(heat_stable)만 의미가 있다. 아니면 NaN.
"""
from __future__ import annotations

from dataclasses import dataclass

from .i2c_bus import I2CBus

NAN = float("nan")


@dataclass
class BmeValue:
    temperature_c: float
    humidity_pct: float
    pressure_hpa: float
    gas_resistance_ohm: float     # heat_stable 이 아니면 NaN
    heat_stable: bool


class BME688:
    def __init__(self, bus: I2CBus, addr: int = 0x76, heater_c: int = 320, heater_ms: int = 150):
        self.bus = bus
        self.addr = addr
        self.heater_c = heater_c
        self.heater_ms = heater_ms
        self.dev = None

    def begin(self) -> str:
        import bme680  # Pi 에서만 필요. 테스트는 이 클래스를 가짜로 바꾼다

        # 라이브러리는 측정 완료를 10 ms x 10 회만 기다린다. 히터 150 ms + 온습압 약 35 ms 면 모자라서
        # 매번 시간 초과가 나므로 기다리는 간격을 늘린다 (히터 150 ms 면 10 x 22 = 220 ms 까지).
        bme680.constants.POLL_PERIOD_MS = max(bme680.constants.POLL_PERIOD_MS, (self.heater_ms + 60) // 10 + 1)

        dev, err = None, None
        for addr in (self.addr, self.addr ^ 0x01):           # 0x76 <-> 0x77 둘 다 시도
            try:
                dev = bme680.BME680(addr, i2c_device=self.bus)
                self.addr = addr
                break
            except (RuntimeError, OSError) as e:
                err = e
        if dev is None:
            raise IOError(f"BME688 없음 (0x76, 0x77): {err}")
        dev.set_humidity_oversample(bme680.OS_2X)
        dev.set_pressure_oversample(bme680.OS_4X)
        dev.set_temperature_oversample(bme680.OS_8X)
        dev.set_filter(bme680.FILTER_SIZE_3)
        dev.set_gas_status(bme680.ENABLE_GAS_MEAS)        # -1: 칩 종류(680/688)에 맞게 자동 선택
        dev.set_gas_heater_temperature(self.heater_c)
        dev.set_gas_heater_duration(self.heater_ms)
        dev.select_gas_heater_profile(0)
        self.dev = dev
        return "BME688" if getattr(dev, "_variant", 0) == 1 else "BME680"

    def read(self) -> BmeValue:
        if self.dev is None:
            raise IOError("BME688 begin() 전")
        if not self.dev.get_sensor_data():
            raise IOError("BME688 측정 시간 초과")
        d = self.dev.data
        return BmeValue(
            temperature_c=float(d.temperature),
            humidity_pct=float(d.humidity),
            pressure_hpa=float(d.pressure),
            gas_resistance_ohm=float(d.gas_resistance) if d.heat_stable else NAN,
            heat_stable=bool(d.heat_stable),
        )
