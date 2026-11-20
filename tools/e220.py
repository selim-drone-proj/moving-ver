#!/usr/bin/env python3
"""E220-900T22D 점검 도구 (Pi5 uart2 = /dev/ttyAMA2 직결)

  python3 e220.py read            설정 읽기
  python3 e220.py setch 72        채널 영구 저장 (주파수 = 850.125 + CH MHz, 국내 920~923 -> 70~72)
  python3 e220.py listen          투명 모드 수신 hex 출력
  python3 e220.py send "hello"    투명 모드 송신
M0 = GPIO23 (16번 핀), M1 = GPIO24 (18번 핀) 을 이 도구가 직접 올리고 내린다 (gpiod v1 의 gpioset, sudo).
명령이 끝나면 항상 투명 모드(M0 = M1 = 0)로 돌려 둔다.
옵션: --port /dev/ttyAMA2  --baud 9600 (설정 모드는 항상 9600 8N1)  --no-gpio (M0/M1 을 손으로 배선한 경우)
"""
import argparse
import subprocess
import sys
import time

import serial

UART = [1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200]
PARITY = ["8N1", "8O1", "8E1", "8N1"]
AIR = ["2.4k", "2.4k", "2.4k", "4.8k", "9.6k", "19.2k", "38.4k", "62.5k"]
SUBPKT = [200, 128, 64, 32]
POWER = ["22 dBm", "17 dBm", "13 dBm", "10 dBm"]


M0_LINE, M1_LINE, CHIP = 23, 24, "gpiochip4"
_hold = None


def set_mode(m0, m1, use_gpio=True):
    """gpioset 을 띄워 둔 동안만 레벨이 유지되므로 프로세스를 잡아 둔다."""
    global _hold
    if not use_gpio:
        return
    if _hold:
        _hold.terminate()
        _hold.wait()
    _hold = subprocess.Popen(["sudo", "gpioset", "-m", "signal", CHIP, f"{M0_LINE}={m0}", f"{M1_LINE}={m1}"])
    time.sleep(0.3)   # 모드 전환 뒤 AUX 가 HIGH 가 될 때까지 (AUX 미배선이라 넉넉히)


def decode(r):
    addh, addl, reg0, reg1, ch, reg3 = r[:6]
    print(f"  주소        : 0x{addh:02X}{addl:02X}  ({addh << 8 | addl})")
    print(f"  UART        : {UART[reg0 >> 5]} bps {PARITY[(reg0 >> 3) & 3]}")
    print(f"  공중 속도   : {AIR[reg0 & 7]} bps")
    print(f"  서브패킷    : {SUBPKT[reg1 >> 6]} B, 주변잡음 RSSI {'ON' if reg1 & 0x20 else 'OFF'}")
    print(f"  출력        : {POWER[reg1 & 3]}")
    print(f"  채널        : {ch}  ->  {850.125 + ch:.3f} MHz")
    print(f"  REG3        : 패킷 RSSI {'ON' if reg3 & 0x80 else 'OFF'}, "
          f"{'고정(주소)' if reg3 & 0x40 else '투명'} 전송, LBT {'ON' if reg3 & 0x10 else 'OFF'}, WOR {reg3 & 7}")
    print(f"  raw         : {r[:6].hex(' ')}")


def cmd_read(a):
    set_mode(1, 1, a.gpio)
    with serial.Serial(a.port, 9600, timeout=1) as s:
        s.reset_input_buffer()
        s.write(bytes([0xC1, 0x00, 0x06]))
        resp = s.read(9)
    if len(resp) == 9 and resp[:3] == bytes([0xC1, 0x00, 0x06]):
        print("E220 응답 OK")
        decode(resp[3:])
        return 0
    print(f"응답 이상: {resp.hex(' ') or '(없음)'}")
    print("  없음 -> 전원(VCC/GND), TX/RX 교차, M0=M1=HIGH 인지 확인")
    print("  'ff ff ff' -> 명령 형식 오류(모드는 설정 모드가 맞음)")
    return 1


def cmd_setch(a):
    ch = int(a.text)
    if not 0 <= ch <= 80:
        print("채널은 0~80")
        return 1
    mhz = 850.125 + ch
    if not 917.0 <= mhz <= 923.5:
        print(f"경고: {mhz:.3f} MHz 는 국내 LPWA 대역(917~923.5 MHz) 밖")
    set_mode(1, 1, a.gpio)
    with serial.Serial(a.port, 9600, timeout=1) as s:
        s.reset_input_buffer()
        s.write(bytes([0xC0, 0x04, 0x01, ch]))      # C0 = 영구 저장
        resp = s.read(4)
    if resp != bytes([0xC1, 0x04, 0x01, ch]):
        print(f"쓰기 실패: {resp.hex(' ') or '(응답 없음)'}")
        return 1
    print(f"채널 {ch} ({mhz:.3f} MHz) 저장됨. 다시 읽기:")
    return cmd_read(a)


def cmd_listen(a):
    set_mode(0, 0, a.gpio)
    print(f"수신 대기 {a.port} @ {a.baud} (Ctrl+C 종료)")
    with serial.Serial(a.port, a.baud, timeout=0.2) as s:
        buf, last = b"", time.time()
        while True:
            d = s.read(256)
            if d:
                buf += d
                last = time.time()
            elif buf and time.time() - last > 0.1:   # 0.1 초 공백 = 한 패킷
                print(f"{time.strftime('%H:%M:%S')} {len(buf):3d}B  {buf.hex(' ')}")
                buf = b""


def cmd_send(a):
    set_mode(0, 0, a.gpio)
    with serial.Serial(a.port, a.baud, timeout=1) as s:
        s.write(a.text.encode())
        s.flush()
    print(f"보냄 {len(a.text)}B")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("mode", choices=["read", "setch", "listen", "send"])
    p.add_argument("text", nargs="?", default="hello e220")
    p.add_argument("--port", default="/dev/ttyAMA2")
    p.add_argument("--baud", type=int, default=9600)
    p.add_argument("--no-gpio", dest="gpio", action="store_false")
    a = p.parse_args()
    try:
        return {"read": cmd_read, "setch": cmd_setch, "listen": cmd_listen, "send": cmd_send}[a.mode](a) or 0
    except KeyboardInterrupt:
        return 0
    finally:
        if a.gpio and a.mode in ("read", "setch"):
            set_mode(0, 0)           # 끝나면 투명 모드로
        if _hold:
            _hold.terminate()
            _hold.wait()


if __name__ == "__main__":
    sys.exit(main())
