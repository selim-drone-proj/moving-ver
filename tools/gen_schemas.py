"""
gen_schemas.py - .msg 에서 JSON Schema 를 만든다 (ROS 밖에서 데이터를 주고받을 때의 규격).

    python tools/gen_schemas.py            # interfaces/schemas/*.schema.json 갱신
    python tools/gen_schemas.py --check    # 파일이 .msg 와 일치하는지만 확인 (CI 용)

ROS -> JSON 변환 규칙 (DATA_SPEC.md 6장과 같음)
  - std_msgs/Header header  ->  "stamp"(ISO 8601 UTC 문자열) + "device_id"
  - builtin_interfaces/Time ->  ISO 8601 UTC 문자열
  - float NaN               ->  null
  - 메시지 안의 메시지       ->  같은 규칙으로 바꾼 객체
  - 최상위 기록에는 "type"(snake_case 메시지 이름)과 "version" 을 붙인다
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from msgspec import INT_RANGES, SPEC_VERSION, Msg, load_all, snake  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "interfaces" / "schemas"
BASE_ID = "https://github.com/selim-drone-proj/moving-ver/interfaces/schemas/"

DEVICE_ID = {
    "type": "string",
    "pattern": "^[a-z][a-z0-9]*_[0-9]{2}$",
    "description": "장비 ID. 소문자 + 밑줄 + 두 자리 번호 (mob_01, fix_01, ess_01, gw_01, octo_01)",
}
STAMP = {
    "type": "string",
    "format": "date-time",
    "pattern": r"Z$",
    "description": "UTC 시각, ISO 8601, 밀리초까지 (예: 2026-10-10T05:12:33.120Z)",
}

# 상수 묶음 -> 그 상수를 쓰는 필드
ENUM_FIELDS = {"LEVEL_": "level", "STATE_": "state", "MODE_": "sampling_mode"}


def field_schema(f, msg: Msg) -> dict:
    t = f.base
    if t == "Time":
        s = dict(STAMP)
    elif t in ("float32", "float64"):
        s = {"type": ["number", "null"]}
    elif t in INT_RANGES:
        lo, hi = INT_RANGES[t]
        s = {"type": "integer", "minimum": lo, "maximum": hi}
    elif t == "bool":
        s = {"type": "boolean"}
    elif t == "string":
        s = {"type": "string"}
    else:  # 다른 odor_msgs 메시지
        s = {"$ref": f"{snake(t)}.schema.json#/$defs/body"}

    if f.is_array:
        s = {"type": "array", "items": s}
        if f.array_len:
            s["minItems"] = s["maxItems"] = f.array_len

    doc = f.doc
    for prefix, fname in ENUM_FIELDS.items():
        if f.name == fname and msg.const_group(prefix):
            group = msg.const_group(prefix)
            s["enum"] = [c.value for c in group]
            doc += " | " + ", ".join(f"{c.value}={c.name[len(prefix):]}" for c in group)
    if f.name == "flags" and msg.const_group("FLAG_"):
        doc += " | 비트: " + ", ".join(f"{c.value}={c.name[5:]}" for c in msg.const_group("FLAG_"))
    if doc and "$ref" not in s:
        s["description"] = doc
    elif doc:
        s = {"allOf": [s], "description": doc}
    return s


def body_schema(msg: Msg) -> dict:
    props, required = {}, []
    for f in msg.fields:
        if f.type == "std_msgs/Header":
            props["stamp"] = dict(STAMP)
            props["device_id"] = dict(DEVICE_ID)
            required += ["stamp", "device_id"]
            continue
        props[f.name] = field_schema(f, msg)
        required.append(f.name)
    return {"type": "object", "properties": props, "required": required, "additionalProperties": False}


def message_schema(msg: Msg) -> dict:
    name = snake(msg.name)
    body = body_schema(msg)
    top_props = {
        "type": {"const": name, "description": "기록 종류"},
        "version": {"const": SPEC_VERSION, "description": "규격 버전"},
    }
    top_props.update(body["properties"])
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": BASE_ID + f"{name}.schema.json",
        "title": msg.name,
        "description": msg.header_doc,
        "type": "object",
        "properties": top_props,
        "required": ["type", "version"] + body["required"],
        "additionalProperties": False,
        "$defs": {"body": body},
    }


def render_all() -> dict[str, str]:
    out = {}
    for msg in load_all().values():
        out[f"{snake(msg.name)}.schema.json"] = json.dumps(message_schema(msg), ensure_ascii=False, indent=2) + "\n"
    return out


def main() -> int:
    check = "--check" in sys.argv
    OUT.mkdir(parents=True, exist_ok=True)
    rendered = render_all()
    stale = []
    for fname, text in rendered.items():
        path = OUT / fname
        if check:
            if not path.exists() or path.read_text(encoding="utf-8") != text:
                stale.append(fname)
        else:
            path.write_text(text, encoding="utf-8")
    if check:
        if stale:
            print("다시 생성 필요:", ", ".join(stale))
            return 1
        print("스키마가 .msg 와 일치")
        return 0
    print("생성:", ", ".join(sorted(rendered)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
