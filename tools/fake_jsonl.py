"""
fake_jsonl.py - 하드웨어, ROS 없이 규격 v0.2 의 가짜 기록 묶음 만들기 (대시보드, 모델 개발용)

    python3 tools/fake_jsonl.py --minutes 30 --out sample_30min.jsonl
    python3 tools/fake_jsonl.py --minutes 10 --fire-event --out fire_demo.jsonl

한 파일에 시간 순서로 섞어서 쓴다 (실제 지상국에서 모이는 모양):
  odor_sample     octo_01 (mob_01 센서)   1 초마다, 반지름 30 m 원 비행, 40 초마다 악취 구름
  station_sample  fix_01 ~ fix_04         60 초마다, 구름이 fix_01 -> fix_04 순서로 4 분 간격 통과
  fire_status     ess_01                  10 초마다 (--fire-event 면 끝 무렵 주의 -> 경고 -> 위험)
  device_health   모든 장비 + gw_01       10 초마다 (고정형은 10 분마다)
모든 가스 기록의 flags 에 SIMULATED(bit11) 가 켜져 있다. 만든 뒤 스키마로 전부 검사한다.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "interfaces" / "python"))
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "odor_drone"))

from odor_json import body, iso_utc, parse_iso, record  # noqa: E402

from odor_drone.fake_values import fake_gas  # noqa: E402

SIM = 2048
NAN = float("nan")
CENTER = (35.8242, 127.1480)
STATIONS = {  # stable-ver/gateway/stations.yaml 의 임시 좌표와 같게
    "fix_01": (35.8250, 127.1470, 42.0), "fix_02": (35.8244, 127.1488, 41.0),
    "fix_03": (35.8236, 127.1478, 40.5), "fix_04": (35.8230, 127.1495, 39.0),
}


def drone(t: float, stamp: float) -> dict:
    ang = 2 * math.pi * t / 60.0
    lat = CENTER[0] + 30 * math.cos(ang) / 111_000.0
    lon = CENTER[1] + 30 * math.sin(ang) / (111_000.0 * math.cos(math.radians(CENTER[0])))
    hdg = (math.degrees(ang) + 90.0) % 360.0
    return body(stamp, "octo_01", armed=True, mode="AUTO", in_air=True, latitude=round(lat, 7),
                longitude=round(lon, 7), altitude_msl=60.0, altitude_rel=20.0, heading_deg=round(hdg, 1),
                gps_fix_type=3, gps_satellites=14, roll_deg=0.0, pitch_deg=0.0, yaw_deg=round(90.0 - hdg, 1),
                ground_speed_mps=3.14, climb_rate_mps=0.0, battery_voltage=round(16.8 - 0.002 * t, 2),
                battery_current=25.0, battery_percent=round(max(0.0, 100.0 - 0.05 * t), 1))


def gas_fields(g: dict) -> dict:
    return {k: (round(v, 4) if isinstance(v, float) and v == v else v) for k, v in g.items()}


def station(no: int, sid: str, t: float, stamp: float, rng: random.Random, seq: int) -> dict:
    phase = (t - (no - 1) * 240.0) % 1800.0
    plume = math.sin(math.pi * phase / 300.0) if phase < 300.0 else 0.0
    lat, lon, alt = STATIONS[sid]
    g = dict(h2s_ppm=round(max(0, 0.01 + 0.4 * plume + rng.gauss(0, 0.005)), 3),
             nh3_ppm=round(max(0, 0.30 + 5.0 * plume + rng.gauss(0, 0.05)), 2), tvoc_ppb=None,
             temperature_c=round(18.0 + rng.gauss(0, 0.2), 2), humidity_pct=round(70.0 + rng.gauss(0, 1), 2),
             pump_on=False, payload_voltage=round(12.1 + rng.gauss(0, 0.02), 2), eco2_ppm=None,
             voc_index=float(round(100 + 250 * plume + rng.gauss(0, 3))), nox_index=1.0,
             pressure_hpa=round(1009.0 + rng.gauss(0, 0.1), 1), gas_resistance_ohm=None,
             pm1_0_ugm3=float(round(8 + rng.gauss(0, 1))), pm2_5_ugm3=float(round(12 + rng.gauss(0, 1))),
             pm10_ugm3=float(round(18 + rng.gauss(0, 2))), nh3_raw=None, h2s_raw=None, flags=SIM)
    return record("station_sample", stamp, sid, latitude=lat, longitude=lon, altitude_msl=alt, sampling_mode=0,
                  avg_window_s=60, gas=body(stamp, sid, **g), seq=seq, rssi_dbm=-60 - no * 8,
                  received=iso_utc(stamp + 1.2))


def fire(t: float, stamp: float, total: float, event: bool, rng: random.Random) -> dict:
    amb = 24.0 + rng.gauss(0, 0.05)
    rise = 0.0
    if event and t > total * 0.6:
        rise = (t - total * 0.6) / (total * 0.4) * 45.0         # 끝에서 최고 약 72 ℃
    px = []
    for r in range(8):
        for c in range(8):
            p = amb + (3.0 if 2 <= r <= 5 and 1 <= c <= 6 else 0.0) + rng.gauss(0, 0.2)
            if (r, c) == (3, 4):
                p += rise
            elif abs(r - 3) + abs(c - 4) == 1:
                p += 0.45 * rise
            px.append(round(p * 4) / 4)
    hot = sum(p > amb + 8.0 for p in px)
    mx = max(px)
    level = 3 if mx >= 60 else 2 if hot >= 3 else 1 if hot else 0
    return record("fire_status", stamp, "ess_01", level=level, latched=level == 3, ambient_temp_c=round(amb, 2),
                  ambient_rh_pct=45.0, surface_max_c=mx, surface_mean_c=round(sum(px) / 64, 3), surface_min_c=min(px),
                  hot_pixels=hot, hotspot_index=px.index(mx), delta_c=8.0, abs_limit_c=60.0, smoke_v=0.80,
                  smoke_baseline_v=0.80, smoke_detected=False, charge_allowed=level < 3, dc_allowed=level < 3,
                  buzzer_on=level >= 2, flags=SIM, thermal_c=px)


def health(stamp: float, did: str, fw: str, up: int, mask: int, rssi: int = 0) -> dict:
    return record("device_health", stamp, did, state=2, firmware=fw, uptime_s=up, sensor_ok_mask=mask,
                  error_count=0, supply_v=None, cpu_temp_c=None, rssi_dbm=rssi, message="simulated")


def generate(minutes: float, start_iso: str, fire_event: bool, seed: int) -> list[dict]:
    rng = random.Random(seed)
    t0 = parse_iso(start_iso)
    total = minutes * 60.0
    out = []
    seq = {sid: 100 for sid in STATIONS}
    for i in range(int(total)):
        t, stamp = float(i), t0 + i
        g = gas_fields(fake_gas(t, rng))
        out.append(record("odor_sample", stamp, "octo_01", drone=drone(t, stamp - 0.04),
                          gas=body(stamp, "mob_01", **g), drone_age_s=0.04))
        for no, sid in enumerate(STATIONS, 1):
            if i % 60 == (no * 7) % 60:
                seq[sid] += 1
                out.append(station(no, sid, t, stamp, rng, seq[sid]))
            if i % 600 == 10 + no:
                out.append(health(stamp + 1.2, sid, "fix_station 0.2.0", 86400 + i, 0b111111, -60 - no * 8))
        if i % 10 == 0:
            out.append(fire(t, stamp, total, fire_event, rng))
            out.append(health(stamp, "mob_01", "odor_drone 0.2.0 (fake)", 3600 + i, 0b11111))
            out.append(health(stamp, "ess_01", "fire_detect 0.2.0 (simulate)", 7200 + i, 0b111))
            out.append(health(stamp, "gw_01", "lora_gateway 0.2.0", 9000 + i, 1))
    out.sort(key=lambda r: r["stamp"])
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--minutes", type=float, default=10.0)
    ap.add_argument("--start", default="2026-10-10T05:00:00.000Z", help="시작 시각 UTC")
    ap.add_argument("--fire-event", action="store_true", help="끝 무렵 화재 감지기 단계 상승")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", help="출력 파일 (없으면 표준 출력)")
    ap.add_argument("--no-check", action="store_true", help="스키마 검사 생략")
    a = ap.parse_args(argv)

    recs = generate(a.minutes, a.start, a.fire_event, a.seed)
    lines = "".join(json.dumps(r, ensure_ascii=False) + "\n" for r in recs)
    if a.out:
        Path(a.out).write_text(lines, encoding="utf-8")
    else:
        sys.stdout.write(lines)
    counts = {}
    for r in recs:
        counts[r["type"]] = counts.get(r["type"], 0) + 1
    if not a.no_check:
        import validate_jsonl
        bad = sum(1 for r in recs if validate_jsonl.errors(r))
        print(f"생성 {len(recs)} 건 {counts}, 스키마 오류 {bad} 건", file=sys.stderr)
        return 1 if bad else 0
    print(f"생성 {len(recs)} 건 {counts}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    sys.exit(main())
