"""
dfrobot_gas.py - DFRobot Gravity 전기화학 가스 센서 (NH3: SEN0469, H2S: SEN0467) I2C 드라이버

제조사 라이브러리(DFRobot_MultiGasSensor, MIT)의 통신 규약을 그대로 옮겼다.
제조사 파이썬 판은 온도 보정을 끄면 0.0 을 돌려주는 버그가 있어 쓰지 않는다.

통신 (기본 주소 0x74, 보드 딥스위치로 0x74~0x77)
  쓰기: 레지스터 0x00 + 9 B  [FF 01 cmd b3 b4 b5 b6 b7 chk]
  (100 ms 대기)
  읽기: 레지스터 0x00 에서 9 B [FF cmd d2 d3 d4 d5 d6 d7 chk]
  chk = (~(b1 + ... + b6) + 1) & 0xFF   <- b7 은 빠진다. 제조사 FucCheckSum(buf, 8) 과 똑같이

명령
  0x78 측정 방식 바꾸기   b3 = 0x03 자동 보고 / 0x04 물어볼 때만(passive)  -> 응답 d2 == 1 이면 성공
  0x86 농도 읽기          -> d2,d3 = 농도(정수), d4 = 가스 종류, d5 = 소수 자릿수(0, 1, 2)
  0x87 보드 온도 읽기      -> d2,d3 = 서미스터 10비트 ADC
  0x91 센서 원전압 읽기    -> d2,d3 -> V = adc * 3.0 / 1024 * 2
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

from .i2c_bus import I2CBus

CMD_CHANGE_GET_METHOD = 0x78
CMD_GET_GAS_CONCENTRATION = 0x86
CMD_GET_TEMP = 0x87
CMD_SENSOR_VOLTAGE = 0x91
MODE_INITIATIVE = 0x03
MODE_PASSIVITY = 0x04

GAS_TYPES = {0x05: "O2", 0x04: "CO", 0x03: "H2S", 0x2C: "NO2", 0x2A: "O3", 0x31: "CL2",
             0x02: "NH3", 0x06: "H2", 0x2E: "HCL", 0x2B: "SO2", 0x33: "HF", 0x45: "PH3"}


class GasProtocolError(IOError):
    """응답 체크섬이 틀리거나 형식이 이상함"""


def checksum(frame: bytes) -> int:
    """제조사 FucCheckSum(buf, 8): 1~6 번째 바이트 합의 2의 보수"""
    return ((~sum(frame[1:7])) + 1) & 0xFF


def pack(cmd: int, *args: int) -> bytes:
    body = [0xFF, 0x01, cmd] + list(args) + [0] * (5 - len(args))
    return bytes(body + [checksum(bytes(body + [0]))])


def check(resp: bytes, cmd: int | None = None) -> bytes:
    if len(resp) != 9 or resp[0] != 0xFF or checksum(resp) != resp[8]:
        raise GasProtocolError(f"응답 체크섬 오류: {bytes(resp).hex(' ')}")
    if cmd is not None and resp[1] != cmd:
        raise GasProtocolError(f"다른 명령의 응답 (기대 0x{cmd:02x}): {bytes(resp).hex(' ')}")
    return resp


def parse_concentration(resp: bytes) -> tuple[float, str]:
    """0x86 응답 -> (ppm, 가스 이름)"""
    value = (resp[2] << 8) | resp[3]
    scale = {0: 1.0, 1: 0.1, 2: 0.01}.get(resp[5], 1.0)
    return value * scale, GAS_TYPES.get(resp[4], f"0x{resp[4]:02x}")


def thermistor_c(adc: int) -> float:
    """보드 서미스터 10비트 ADC -> ℃ (제조사 식: 10 kΩ, B = 3380.13, 3 V 기준)"""
    v = 3.0 * adc / 1024.0
    if not 0.0 < v < 3.0:
        return float("nan")
    r = v * 10000.0 / (3.0 - v)
    return 1.0 / (1.0 / (273.15 + 25.0) + 1.0 / 3380.13 * math.log(r / 10000.0)) - 273.15


def vendor_temp_correct(gas: str, ppm: float, t: float) -> float | None:
    """제조사 C++ 라이브러리의 온도 보정식 (NH3, H2S 만). 식의 적용 범위 밖이면 None.

    주의: H2S 의 20 ℃ 초과 구간 식 ppm / (0.015 t - 0.3) 은 20 ℃ 에서 0 으로 나누게 되고
    25 ℃ 에서 13 배가 된다. 제조사 식 그대로 옮겼지만 믿기 어려우므로
    gas_reader 는 기본으로 온도 보정을 쓰지 않는다 (DATA_SPEC.md 4.2 참고).
    """
    if t is None or math.isnan(t):
        return None
    if gas == "NH3":
        if -20 < t <= 0:
            c = ppm / (0.006 * t + 0.95) - (-0.006 * t + 0.25)
        elif 0 < t <= 20:
            c = ppm / (0.006 * t + 0.95) - (-0.012 * t + 0.25)
        elif 20 < t <= 40:
            c = ppm / (0.005 * t + 1.08) - (-0.1 * t + 2)
        else:
            return None
    elif gas == "H2S":
        if -20 < t <= 20:
            c = ppm / (0.005 * t + 0.92)
        elif 20 < t <= 60:
            c = ppm / (0.015 * t - 0.3)
        else:
            return None
    else:
        return None
    return c if c >= 1e-5 else 0.0


@dataclass
class GasValue:
    ppm: float          # 센서가 낸 값 (온도 보정 전)
    gas: str            # 센서가 알려 준 가스 종류


class GravityGas:
    def __init__(self, bus: I2CBus, addr: int = 0x74, expect: str | None = None,
                 settle_s: float = 0.1, sleep=time.sleep):
        self.bus = bus
        self.addr = addr
        self.expect = expect          # "NH3" 또는 "H2S". 다른 센서가 꽂혀 있으면 begin() 에서 알려 줌
        self.settle_s = settle_s
        self.sleep = sleep
        self.gas: str | None = None

    def _cmd(self, cmd: int, *args: int) -> bytes:
        self.bus.write(self.addr, b"\x00" + pack(cmd, *args))
        self.sleep(self.settle_s)
        return check(self.bus.write_read(self.addr, b"\x00", 9))

    def begin(self) -> str:
        """passive 모드로 바꾸고 가스 종류를 확인한다. 실패하면 예외"""
        resp = self._cmd(CMD_CHANGE_GET_METHOD, MODE_PASSIVITY)
        if resp[2] != 1:
            raise GasProtocolError(f"0x{self.addr:02x} passive 모드 전환 거부: {resp.hex(' ')}")
        _, gas = parse_concentration(self._cmd(CMD_GET_GAS_CONCENTRATION))
        if self.expect and gas != self.expect:
            raise GasProtocolError(f"{self.expect} 자리에 {gas} 센서가 연결됨 (채널, 주소 확인)")
        self.gas = gas
        return gas

    def read(self) -> GasValue:
        ppm, gas = parse_concentration(self._cmd(CMD_GET_GAS_CONCENTRATION))
        if self.expect and gas != self.expect:
            raise GasProtocolError(f"{self.expect} 자리에서 {gas} 응답")
        return GasValue(ppm, gas)

    def read_temp_c(self) -> float:
        resp = self._cmd(CMD_GET_TEMP)
        return thermistor_c((resp[2] << 8) | resp[3])

    def read_voltage(self) -> float:
        resp = self._cmd(CMD_SENSOR_VOLTAGE)
        return ((resp[2] << 8) | resp[3]) * 3.0 / 1024 * 2
