"""
fake_values.py - 가짜 이동형 가스 값 (fake_gas_node, tools/fake_jsonl.py 가 같이 쓴다)

배경 농도 + 잡음, 그리고 40 초마다 10 초짜리 악취 구름(plume) 통과.
실제 이동형(gas_i2c_node)이 채우는 필드만 채우고 나머지는 NaN. flags 에는 항상 FLAG_SIMULATED.
"""
from __future__ import annotations

import math
import random

from .gas_reader import FLAG_SIMULATED

NAN = float("nan")
PERIOD_S = 40.0
PLUME_S = 10.0


def plume(t: float) -> float:
    """0~1 산 모양. 40 초 주기의 처음 10 초"""
    phase = t % PERIOD_S
    return math.sin(math.pi * phase / PLUME_S) if phase < PLUME_S else 0.0


def fake_gas(t: float, rng: random.Random | None = None) -> dict:
    rng = rng or random
    p = plume(t)
    nh3 = max(0.0, 0.50 + 6.0 * p + rng.gauss(0, 0.05))
    h2s = max(0.0, 0.02 + 0.8 * p + rng.gauss(0, 0.01))
    tvoc = max(0.0, round(120.0 + 900.0 * p + rng.gauss(0, 10.0)))
    return dict(
        h2s_ppm=h2s, nh3_ppm=nh3, tvoc_ppb=float(tvoc),
        temperature_c=27.0 + rng.gauss(0, 0.1), humidity_pct=60.0 + rng.gauss(0, 0.5),
        pump_on=False, payload_voltage=NAN,
        eco2_ppm=float(round(400.0 + 0.8 * tvoc)), voc_index=NAN, nox_index=NAN,
        pressure_hpa=1008.5 + rng.gauss(0, 0.05), gas_resistance_ohm=60000.0 / (1.0 + 3.0 * p),
        pm1_0_ugm3=NAN, pm2_5_ugm3=NAN, pm10_ugm3=NAN,
        nh3_raw=nh3, h2s_raw=h2s,
        flags=FLAG_SIMULATED,
    )
