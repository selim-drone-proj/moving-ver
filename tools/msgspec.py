"""
msgspec.py - odor_msgs 의 .msg 파일을 읽어 필드 목록으로 바꾸는 작은 파서.

.msg 가 규격의 단일 원본이다. JSON Schema(gen_schemas.py)와 규격 문서의 필드 표(gen_docs.py)는
전부 여기서 만든다. 손으로 고치지 말고 .msg 를 고친 뒤 다시 생성할 것.

ROS 없이 표준 라이브러리만으로 돈다.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

MSG_DIR = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "odor_msgs" / "msg"
SPEC_VERSION = "0.2"

PRIMITIVES = {
    "bool", "byte", "char",
    "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64",
    "float32", "float64", "string",
}
INT_RANGES = {
    "int8": (-128, 127), "uint8": (0, 255),
    "int16": (-32768, 32767), "uint16": (0, 65535),
    "int32": (-2**31, 2**31 - 1), "uint32": (0, 2**32 - 1),
    "int64": (-2**63, 2**63 - 1), "uint64": (0, 2**64 - 1),
}

FIELD_RE = re.compile(r"^(?P<type>[A-Za-z0-9_/]+)(?P<arr>\[\d*\])?\s+(?P<name>[a-z][a-z0-9_]*)\s*(?:#\s*(?P<doc>.*))?$")
CONST_RE = re.compile(r"^(?P<type>[a-z0-9]+)\s+(?P<name>[A-Z][A-Z0-9_]*)\s*=\s*(?P<value>[^#\s]+)\s*(?:#\s*(?P<doc>.*))?$")


@dataclass
class Field:
    type: str           # 예: float32, GasReading, std_msgs/Header
    name: str
    doc: str = ""
    array_len: int | None = None   # 고정 길이 배열이면 길이
    is_array: bool = False

    @property
    def base(self) -> str:
        return self.type.split("/")[-1]


@dataclass
class Const:
    type: str
    name: str
    value: int
    doc: str = ""


@dataclass
class Msg:
    name: str
    header_doc: str = ""
    fields: list[Field] = field(default_factory=list)
    consts: list[Const] = field(default_factory=list)

    def const_group(self, prefix: str) -> list[Const]:
        return [c for c in self.consts if c.name.startswith(prefix)]


def parse_msg(path: Path) -> Msg:
    msg = Msg(name=path.stem)
    lead: list[str] = []
    in_lead = True
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line:
            in_lead = False if msg.fields or msg.consts else in_lead
            continue
        if line.startswith("#"):
            if in_lead and not msg.fields and not msg.consts:
                lead.append(line.lstrip("# ").rstrip())
            continue
        in_lead = False
        m = CONST_RE.match(line)
        if m:
            msg.consts.append(Const(m["type"], m["name"], int(m["value"], 0), (m["doc"] or "").strip()))
            continue
        m = FIELD_RE.match(line)
        if not m:
            raise ValueError(f"{path.name}: 해석할 수 없는 줄: {raw!r}")
        arr = m["arr"]
        f = Field(type=m["type"], name=m["name"], doc=(m["doc"] or "").strip())
        if arr:
            f.is_array = True
            n = arr.strip("[]")
            f.array_len = int(n) if n else None
        msg.fields.append(f)
    msg.header_doc = " ".join(lead)
    return msg


def load_all(msg_dir: Path = MSG_DIR) -> dict[str, Msg]:
    return {p.stem: parse_msg(p) for p in sorted(msg_dir.glob("*.msg"))}


def validate_names(msgs: dict[str, Msg]) -> list[str]:
    """ROS 2 이름 규칙과 타입 존재 여부를 검사. 문제 목록을 돌려준다."""
    problems = []
    known = set(msgs) | {"std_msgs/Header", "builtin_interfaces/Time"}
    for m in msgs.values():
        seen = set()
        for f in m.fields:
            if "__" in f.name or f.name.endswith("_"):
                problems.append(f"{m.name}.{f.name}: 밑줄 규칙 위반")
            if f.name in seen:
                problems.append(f"{m.name}.{f.name}: 이름 중복")
            seen.add(f.name)
            if f.type not in PRIMITIVES and f.type not in known and f.base not in msgs:
                problems.append(f"{m.name}.{f.name}: 모르는 타입 {f.type}")
        for c in m.consts:
            if c.type not in INT_RANGES:
                problems.append(f"{m.name}.{c.name}: 상수는 정수 타입만 씀")
            else:
                lo, hi = INT_RANGES[c.type]
                if not lo <= c.value <= hi:
                    problems.append(f"{m.name}.{c.name}: {c.value} 가 {c.type} 범위 밖")
    return problems


def snake(name: str) -> str:
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


if __name__ == "__main__":
    msgs = load_all()
    for m in msgs.values():
        print(f"{m.name}: 필드 {len(m.fields)}, 상수 {len(m.consts)}")
    probs = validate_names(msgs)
    print("문제:", probs or "없음")
