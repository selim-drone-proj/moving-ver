"""
validate_jsonl.py - JSON 기록이 규격 v0.2 스키마에 맞는지 검사한다.

    python tools/validate_jsonl.py ~/odor_logs/gateway_20261010.jsonl
    python tools/validate_jsonl.py interfaces/examples/*.json

각 기록의 "type" 으로 스키마를 골라 검사하고, 틀린 줄과 이유를 출력한다.
소프트웨어 쪽에서 데이터를 만들거나 받을 때도 이걸로 확인하면 된다.
필요: pip install jsonschema
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

from jsonschema import Draft202012Validator
from referencing import Registry, Resource

SCHEMA_DIR = Path(__file__).resolve().parents[1] / "interfaces" / "schemas"


def load_registry(schema_dir: Path = SCHEMA_DIR) -> tuple[Registry, dict[str, dict]]:
    schemas = {}
    resources = []
    for p in sorted(schema_dir.glob("*.schema.json")):
        s = json.loads(p.read_text(encoding="utf-8"))
        schemas[p.name.replace(".schema.json", "")] = s
        resources.append((s["$id"], Resource.from_contents(s)))
    return Registry().with_resources(resources), schemas


_REG, _SCHEMAS = None, None


def validator_for(type_name: str, schema_dir: Path = SCHEMA_DIR) -> Draft202012Validator:
    global _REG, _SCHEMAS
    if _REG is None or schema_dir != SCHEMA_DIR:
        _REG, _SCHEMAS = load_registry(schema_dir)
    if type_name not in _SCHEMAS:
        raise KeyError(f"모르는 기록 종류: {type_name}")
    return Draft202012Validator(_SCHEMAS[type_name], registry=_REG,
                                format_checker=Draft202012Validator.FORMAT_CHECKER)


def errors(rec: dict, schema_dir: Path = SCHEMA_DIR) -> list[str]:
    t = rec.get("type")
    if not t:
        return ["'type' 필드가 없음"]
    try:
        v = validator_for(t, schema_dir)
    except KeyError as e:
        return [str(e)]
    return [f"{'/'.join(map(str, e.absolute_path)) or '(최상위)'}: {e.message}" for e in v.iter_errors(rec)]


def iter_records(path: Path):
    text = path.read_text(encoding="utf-8")
    if path.suffix == ".json":
        data = json.loads(text)
        for i, rec in enumerate(data if isinstance(data, list) else [data], 1):
            yield i, rec
    else:
        for i, line in enumerate(text.splitlines(), 1):
            if line.strip():
                yield i, json.loads(line)


def main(argv: list[str]) -> int:
    if not argv:
        print(__doc__)
        return 2
    bad = total = 0
    for name in argv:
        for line_no, rec in iter_records(Path(name)):
            total += 1
            errs = errors(rec)
            if errs:
                bad += 1
                print(f"{name}:{line_no} [{rec.get('type')}]")
                for e in errs[:5]:
                    print("   -", e)
    print(f"검사 {total}건, 오류 {bad}건")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
