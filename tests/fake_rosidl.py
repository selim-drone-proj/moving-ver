"""
fake_rosidl.py - ROS 2 가 없는 PC 에서 odor_msgs 메시지 클래스를 흉내 낸다 (테스트 전용).

.msg 를 읽어 rclpy 가 만드는 클래스와 같은 모양(get_fields_and_field_types, 기본값, 중첩 메시지)의
클래스를 만든다. 타입 이름도 rclpy 와 같은 IDL 이름을 쓴다 (float32 -> 'float', float64 -> 'double').
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from msgspec import load_all  # noqa: E402

IDL = {"bool": "boolean", "float32": "float", "float64": "double", "byte": "octet"}


class Time:
    @classmethod
    def get_fields_and_field_types(cls):
        return {"sec": "int32", "nanosec": "uint32"}

    def __init__(self):
        self.sec = 0
        self.nanosec = 0


class Header:
    @classmethod
    def get_fields_and_field_types(cls):
        return {"stamp": "builtin_interfaces/Time", "frame_id": "string"}

    def __init__(self):
        self.stamp = Time()
        self.frame_id = ""


def _idl(ros_type: str, msgs) -> str:
    if ros_type == "std_msgs/Header":
        return "std_msgs/Header"
    if ros_type == "builtin_interfaces/Time":
        return "builtin_interfaces/Time"
    base = ros_type.split("/")[-1]
    if base in msgs:
        return f"odor_msgs/{base}"
    return IDL.get(ros_type, ros_type)


def build_classes():
    msgs = load_all()
    classes = {}

    def default(f):
        t = f.type
        if t == "std_msgs/Header":
            return Header()
        if t == "builtin_interfaces/Time":
            return Time()
        if f.base in msgs:
            return classes[f.base]()
        if f.is_array:
            return [0.0] * (f.array_len or 0)
        if t == "bool":
            return False
        if t == "string":
            return ""
        if t.startswith("float"):
            return 0.0
        return 0

    # 의존 순서대로 (DroneState, GasReading 이 먼저)
    order = sorted(msgs, key=lambda n: sum(1 for f in msgs[n].fields if f.base in msgs))
    for name in order:
        m = msgs[name]
        types = {}
        for f in m.fields:
            t = _idl(f.type, msgs)
            if f.is_array:
                t = f"{t}[{f.array_len}]" if f.array_len else f"sequence<{t}>"
            types[f.name] = t

        def make(m=m, types=types):
            class Cls:
                __slots__ = tuple(types)

                @classmethod
                def get_fields_and_field_types(cls):
                    return dict(types)

                def __init__(self):
                    for f in m.fields:
                        setattr(self, f.name, default(f))
            Cls.__name__ = m.name
            for c in m.consts:
                setattr(Cls, c.name, c.value)
            return Cls

        classes[name] = make()
    return classes
