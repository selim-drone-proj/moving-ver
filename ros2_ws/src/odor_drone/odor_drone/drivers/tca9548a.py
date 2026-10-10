"""
tca9548a.py - I2C 멀티플렉서 TCA9548A (기본 주소 0x70)

Gravity NH3, H2S 는 둘 다 기본 주소 0x74 라서 같은 버스에 그냥 붙이면 충돌한다.
TCA9548A 의 채널을 나눠 붙이고, 센서마다 ChannelBus 를 하나씩 준다.
ChannelBus 는 전송 직전에 자기 채널을 켜므로 드라이버는 멀티플렉서를 몰라도 된다.

    tca = TCA9548A(bus)
    nh3 = GravityGas(tca.channel(0), 0x74)
    h2s = GravityGas(tca.channel(1), 0x74)

(Gravity 보드 뒷면 딥스위치로 주소를 0x74~0x77 중 다르게 바꾸면 멀티플렉서 없이도 된다.
 그 경우 설정의 tca_address 를 -1 로.)
"""
from __future__ import annotations

from .i2c_bus import I2CBus


class TCA9548A:
    def __init__(self, bus: I2CBus, addr: int = 0x70):
        self.bus = bus
        self.addr = addr
        self._current: int | None = None   # 마지막으로 켠 채널 (같으면 다시 안 씀)

    def select(self, ch: int) -> None:
        if not 0 <= ch <= 7:
            raise ValueError(f"TCA9548A 채널은 0~7: {ch}")
        if ch != self._current:
            try:
                self.bus.write(self.addr, bytes([1 << ch]))
            except OSError:
                self._current = None
                raise
            self._current = ch

    def disable_all(self) -> None:
        self.bus.write(self.addr, b"\x00")
        self._current = None

    def forget(self) -> None:
        """다음 전송에서 채널을 반드시 다시 쓰게 한다 (오류 뒤, 다른 프로그램이 건드렸을 때)"""
        self._current = None

    def probe(self) -> bool:
        try:
            self.bus.read(self.addr, 1)
            return True
        except OSError:
            return False

    def channel(self, ch: int) -> "ChannelBus":
        return ChannelBus(self, ch)


class ChannelBus(I2CBus):
    """멀티플렉서의 채널 하나를 독립된 버스처럼 쓰게 해 준다"""

    def __init__(self, mux: TCA9548A, ch: int):
        self.mux = mux
        self.ch = ch

    def _guard(self, fn, *args):
        self.mux.select(self.ch)
        try:
            return fn(*args)
        except OSError:
            self.mux.forget()
            raise

    def write(self, addr, data):
        return self._guard(self.mux.bus.write, addr, data)

    def read(self, addr, n):
        return self._guard(self.mux.bus.read, addr, n)

    def write_read(self, addr, data, n):
        return self._guard(self.mux.bus.write_read, addr, data, n)
