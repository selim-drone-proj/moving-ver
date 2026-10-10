"""
gen_docs.py - docs/DATA_SPEC.md 안의 필드 표와 상수 표를 .msg 에서 다시 만든다.

    python3 tools/gen_docs.py           # 문서 갱신
    python3 tools/gen_docs.py --check   # 문서가 .msg 와 맞는지만 검사 (테스트에서 사용)

문서 안의 아래 표시 사이를 통째로 바꾼다. 표시 밖은 손으로 쓴 설명이라 건드리지 않는다.
    <!-- gen:fields GasReading -->  ...  <!-- /gen -->
    <!-- gen:consts GasReading FLAG_ -->  ...  <!-- /gen -->
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

from msgspec import load_all

DOC = Path(__file__).resolve().parents[1] / "docs" / "DATA_SPEC.md"
BLOCK_RE = re.compile(r"(<!-- gen:(?P<kind>fields|consts) (?P<args>[^>]*?) -->\n)(?P<body>.*?)(<!-- /gen -->)", re.S)

JSON_TYPE = {
    "bool": "boolean", "string": "string", "float32": "number \\| null", "float64": "number \\| null",
    "uint8": "integer 0~255", "uint16": "integer 0~65535", "uint32": "integer", "int16": "integer", "int32": "integer",
}


def esc(s: str) -> str:
    return s.replace("|", "\\|")


def fields_table(msgs, name: str) -> str:
    m = msgs[name]
    rows = ["| 필드 (JSON 키) | ROS 타입 | JSON 타입 | 설명 |", "|---|---|---|---|"]
    for f in m.fields:
        if f.type == "std_msgs/Header":
            rows.append(f"| `stamp` | header.stamp | string (UTC ISO 8601) | {esc(f.doc)} |")
            rows.append("| `device_id` | header.frame_id | string | 장비 ID (2.1) |")
            continue
        rtype = f.type + (f"[{f.array_len or ''}]" if f.is_array else "")
        if f.type == "builtin_interfaces/Time":
            jtype = "string (UTC ISO 8601)"
        elif f.base in msgs:
            jtype = f"object ({f.base}, [{f.base}](#{f.base.lower()}))"
        elif f.is_array:
            jtype = f"array[{f.array_len}] of " + JSON_TYPE.get(f.type, f.type)
        else:
            jtype = JSON_TYPE.get(f.type, f.type)
        rows.append(f"| `{f.name}` | {rtype} | {jtype} | {esc(f.doc)} |")
    return "\n".join(rows) + "\n"


def consts_table(msgs, name: str, prefix: str) -> str:
    rows = ["| 상수 | 값 | 비트 | 뜻 |", "|---|---|---|---|"]
    for c in msgs[name].const_group(prefix):
        bit = f"bit{c.value.bit_length() - 1}" if prefix == "FLAG_" else ""
        doc = re.sub(r"^bit\d+\s+", "", c.doc)
        rows.append(f"| `{c.name}` | {c.value} | {bit} | {esc(doc)} |")
    return "\n".join(rows) + "\n"


def render(text: str) -> str:
    msgs = load_all()

    def sub(m):
        args = m["args"].split()
        if m["kind"] == "fields":
            body = fields_table(msgs, args[0])
        else:
            body = consts_table(msgs, args[0], args[1])
        return m.group(1) + body + m.group(5)

    return BLOCK_RE.sub(sub, text)


def main(argv: list[str]) -> int:
    text = DOC.read_text(encoding="utf-8")
    new = render(text)
    if "--check" in argv:
        if new != text:
            print("DATA_SPEC.md 의 표가 .msg 와 다름. python3 tools/gen_docs.py 로 다시 만들 것")
            return 1
        print("DATA_SPEC.md 표가 .msg 와 일치")
        return 0
    DOC.write_text(new, encoding="utf-8")
    n = len(BLOCK_RE.findall(new))
    print(f"DATA_SPEC.md 표 {n} 개 갱신")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
