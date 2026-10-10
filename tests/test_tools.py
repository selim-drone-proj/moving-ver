"""도구와 공통 파일 검사: 가짜 기록 묶음이 규격에 맞는가, 세 저장소의 공통 파일이 같은가"""
import filecmp
import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT.parent                     # moving-ver, stable-ver, fire_detect 를 나란히 받아 둔 폴더


def test_fake_jsonl_is_valid_and_ordered(tmp_path):
    out = tmp_path / "s.jsonl"
    r = subprocess.run([sys.executable, str(ROOT / "tools" / "fake_jsonl.py"), "--minutes", "3", "--fire-event",
                        "--out", str(out)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    recs = [json.loads(x) for x in out.read_text(encoding="utf-8").splitlines()]
    assert {x["type"] for x in recs} == {"odor_sample", "station_sample", "fire_status", "device_health"}
    stamps = [x["stamp"] for x in recs]
    assert stamps == sorted(stamps)
    assert all(x["gas"]["flags"] & 2048 for x in recs if "gas" in x)


SHARED = [
    ("interfaces/python/odor_json.py", "ros2_ws/src/odor_drone/odor_drone/odor_json.py"),
    ("interfaces/python/odor_json.py", "../stable-ver/gateway/odor_gateway/odor_json.py"),
    ("interfaces/python/odor_json.py", "../fire_detect/fire_detect/odor_json.py"),
    ("ros2_ws/src/odor_drone/odor_drone/drivers/i2c_bus.py", "../fire_detect/fire_detect/drivers/i2c_bus.py"),
    ("ros2_ws/src/odor_drone/odor_drone/drivers/sensirion.py", "../fire_detect/fire_detect/drivers/sensirion.py"),
    ("ros2_ws/src/odor_drone/odor_drone/system_info.py", "../fire_detect/fire_detect/system_info.py"),
]


@pytest.mark.parametrize("src,copy", SHARED, ids=lambda s: s.split("/")[-1])
def test_shared_files_are_identical(src, copy):
    a, b = ROOT / src, (ROOT / copy).resolve()
    if not b.exists():
        pytest.skip(f"{b} 없음 (다른 저장소를 옆에 받지 않음)")
    assert filecmp.cmp(a, b, shallow=False), f"{copy} 가 원본 {src} 와 다름. 원본을 복사할 것"


def test_spec_doc_tables_match_msg():
    r = subprocess.run([sys.executable, str(ROOT / "tools" / "gen_docs.py"), "--check"], capture_output=True,
                       text=True, cwd=ROOT / "tools")
    assert r.returncode == 0, r.stdout
