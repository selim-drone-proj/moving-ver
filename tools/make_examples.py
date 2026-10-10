"""규격 문서와 SW 팀이 참고할 예시 기록을 만든다 (interfaces/examples/*.json)."""
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "interfaces" / "python"))
from odor_json import body, iso_utc, record  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "interfaces" / "examples"
T = 1791609153.12   # 2026-10-10T05:12:33.120Z

drone = body(T - 0.04, "octo_01", armed=True, mode="AUTO", in_air=True,
             latitude=35.8243715, longitude=127.1486021, altitude_msl=60.2, altitude_rel=20.1,
             heading_deg=87.5, gps_fix_type=3, gps_satellites=15, roll_deg=1.2, pitch_deg=-2.3, yaw_deg=2.5,
             ground_speed_mps=3.1, climb_rate_mps=0.0, battery_voltage=23.4, battery_current=41.2,
             battery_percent=78.0)
gas_mob = body(T, "mob_01", h2s_ppm=0.021, nh3_ppm=0.83, tvoc_ppb=240.0, temperature_c=23.1, humidity_pct=58.4,
               pump_on=False, payload_voltage=None, eco2_ppm=520.0, voc_index=None, nox_index=None,
               pressure_hpa=1008.4, gas_resistance_ohm=52300.0, pm1_0_ugm3=None, pm2_5_ugm3=None, pm10_ugm3=None,
               nh3_raw=0.7, h2s_raw=0.02, flags=1024)
gas_fix = body(T, "fix_01", h2s_ppm=0.012, nh3_ppm=1.23, tvoc_ppb=None, temperature_c=18.62, humidity_pct=71.3,
               pump_on=False, payload_voltage=None, eco2_ppm=None, voc_index=112.0, nox_index=1.0,
               pressure_hpa=None, gas_resistance_ohm=None, pm1_0_ugm3=8.0, pm2_5_ugm3=13.0, pm10_ugm3=21.0,
               nh3_raw=14760.0, h2s_raw=240.0, flags=0)
thermal = [round(24.0 + 9.0 * math.exp(-((r - 2) ** 2 + (c - 5) ** 2) / 2.0), 2) for r in range(8) for c in range(8)]

examples = {
    "drone_state": record("drone_state", T - 0.04, "octo_01", **{k: v for k, v in drone.items() if k not in ("stamp", "device_id")}),
    "gas_reading": record("gas_reading", T, "mob_01", **{k: v for k, v in gas_mob.items() if k not in ("stamp", "device_id")}),
    "odor_sample": record("odor_sample", T, "octo_01", drone=drone, gas=gas_mob, drone_age_s=0.04),
    "station_sample": record("station_sample", T, "fix_01", latitude=35.8242, longitude=127.148, altitude_msl=40.0,
                             sampling_mode=0, avg_window_s=60, gas=gas_fix, seq=1287, rssi_dbm=-87,
                             received=iso_utc(T + 1.3)),
    "fire_status": record("fire_status", T, "ess_01", level=1, latched=False, ambient_temp_c=23.8, ambient_rh_pct=46.0,
                          surface_max_c=max(thermal), surface_mean_c=round(sum(thermal) / 64, 2),
                          surface_min_c=min(thermal), hot_pixels=sum(x > 23.8 + 6.0 for x in thermal),
                          hotspot_index=thermal.index(max(thermal)), delta_c=6.0,
                          abs_limit_c=60.0, smoke_v=0.62, smoke_baseline_v=0.6, smoke_detected=False,
                          charge_allowed=True, dc_allowed=True, buzzer_on=False, flags=0, thermal_c=thermal),
    "device_health": record("device_health", T, "fix_01", state=2, firmware="fix_station 0.2.0", uptime_s=86400,
                            sensor_ok_mask=63, error_count=0, supply_v=12.05, cpu_temp_c=None, rssi_dbm=-87,
                            message=""),
}
OUT.mkdir(parents=True, exist_ok=True)
for name, rec in examples.items():
    (OUT / f"{name}.json").write_text(json.dumps(rec, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print("예시:", ", ".join(sorted(examples)))
