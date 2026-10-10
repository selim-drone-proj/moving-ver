# moving-ver: 데이터 규격 v0.2 + 이동형 악취 드론 (RPi5, ROS 2 Jazzy, Pixhawk)

이 저장소에는 두 가지가 들어 있습니다.

1. **세 장비 공통 데이터 규격** (이동형, 고정형, 화재 감지기). 소프트웨어 팀은 [`docs/DATA_SPEC.md`](docs/DATA_SPEC.md) 부터 보십시오.
2. **이동형 온보드 ROS 2 워크스페이스**. Pixhawk 데이터와 악취 센서 값을 지오태깅해서 넘깁니다.

v0.1 은 [YoungHyun1884/drone_stink](https://github.com/YoungHyun1884/drone_stink) (MIT) 입니다.
v0.2 는 그 구조와 토픽을 그대로 두고 필드와 노드를 덧붙였습니다. 바뀐 점은 [`docs/CHANGES.md`](docs/CHANGES.md).

```
moving-ver/
├── docs/DATA_SPEC.md          규격 문서 (필드 표는 .msg 에서 자동 생성)
├── docs/CHANGES.md            v0.1 -> v0.2 소프트웨어 변경
├── interfaces/
│   ├── schemas/               JSON Schema 6 종 (자동 생성)
│   ├── examples/              종류별 예시 기록
│   ├── samples/               10 분짜리 가짜 기록 묶음 (fake_jsonl.py 로 만든 것)
│   └── python/odor_json.py    JSON <-> ROS 변환 (세 저장소 공통 파일)
├── tools/                     gen_schemas.py, gen_docs.py, validate_jsonl.py, fake_jsonl.py
├── tests/                     규격 자체의 검사 (ROS 없이)
└── ros2_ws/src/
    ├── odor_msgs/             메시지 6 종 (규격의 단일 원본)
    └── odor_drone/            이동형 노드, 드라이버, 런치, 설정, 테스트
```

## 1. 하드웨어 없이 바로 써 보기 (소프트웨어 팀)

```bash
pip install "jsonschema>=4.18"
python3 tools/fake_jsonl.py --minutes 30 --fire-event --out sample.jsonl   # 세 장비 기록이 섞인 30 분
python3 tools/validate_jsonl.py sample.jsonl                               # 규격 검사
```

ROS 2 로 받고 싶으면 워크스페이스를 빌드하고 가짜 노드를 띄웁니다 (3장 설치 뒤).

```bash
ros2 launch odor_drone fake.launch.py
ros2 topic echo --once /odor/sample
```

## 2. 노드 구조 (이동형)

```
 Pixhawk ──TELEM2(MAVLink)──▶ mavros ──/mavros/*──▶ fc_state_node ──/drone/state (10 Hz)──┐
                                                                                         ├─▶ odor_sample_node ──/odor/sample (1 Hz)──▶ 모델, 대시보드
 센서 4종 ──I2C (TCA9548A)──▶ gas_i2c_node ──/odor/gas (1 Hz)────────────────────────────┘                  │
                               └──/health/mob_01 (10 초)                                                    └─▶ csv_logger_node ──▶ ~/odor_logs/*.csv, *.jsonl
```

| 노드 | 하는 일 | 구독 | 발행 |
|---|---|---|---|
| `mavros` (외부) | Pixhawk MAVLink <-> ROS 2 | | `/mavros/*` |
| `fc_state_node` | mavros 토픽 9 개를 모아 10 Hz 로 한 메시지 | `/mavros/*` | `/drone/state` |
| `gas_i2c_node` | **v0.2 새 노드.** 센서 4 종을 Pi5 I2C 로 직접 읽음 | (I2C) | `/odor/gas`, `/health/mob_01` |
| `gas_sensor_node` | 아두이노 모듈의 시리얼 한 줄 (v0.1 방식, 남겨 둠) | (시리얼) | `/odor/gas` |
| `odor_sample_node` | 가스 값마다 그 순간 드론 상태를 붙임 | `/drone/state`, `/odor/gas` | `/odor/sample` |
| `csv_logger_node` | CSV 와 JSONL 로 로컬 저장 | `/odor/sample` | (파일) |
| `fake_fc_node`, `fake_gas_node` | 하드웨어 없이 같은 토픽을 가짜로 | | `/drone/state`, `/odor/gas` |

v0.1 은 ROS 기능을 pub/sub 와 타이머만 썼습니다. v0.2 는 여기에 **파라미터**만 더해 장비 ID, 센서 주소를
`config/mob_01.yaml` 에서 바꿀 수 있게 했습니다. 서비스, 액션은 여전히 쓰지 않습니다.

## 3. 센서 배선 (mob_01)

결선 도면(2026-10-05) 기준. 센서는 드론 붐에 노출 장착하고, 전자부만 방수 함체에 넣습니다.

| TCA9548A 채널 | 센서 | 주소 | 채우는 필드 |
|---|---|---|---|
| CH0 | Gravity NH3 SEN0469 | 0x74 | nh3_ppm, nh3_raw |
| CH1 | Gravity H2S SEN0467 | 0x74 | h2s_ppm, h2s_raw |
| CH2 | Grove SGP30 | 0x58 | tvoc_ppb, eco2_ppm |
| CH3 | Gravity BME688 | 0x76 (안 되면 0x77 자동) | temperature_c, humidity_pct, pressure_hpa, gas_resistance_ohm |

- TCA9548A (0x70) 는 Pi5 40핀 1번(3.3 V), 3번(SDA), 5번(SCL), 6번(GND).
- Gravity 두 개가 같은 0x74 라서 멀티플렉서로 나눕니다. 보드 뒷면 딥스위치로 주소를 다르게 하면
  멀티플렉서 없이 직결해도 되고, 그때는 `tca_address: -1` (BME688 이 0x76, 0x77 을 쓰므로 Gravity 는 0x74, 0x75 로).
- 채널을 바꿔 꽂으면 노드가 "NH3 자리에 H2S 센서가 연결됨" 이라고 알려 줍니다.
- I2C 켜기: `/boot/firmware/config.txt` 에 `dtparam=i2c_arm=on`, 재부팅 후 `sudo i2cdetect -y 1` 에 `70` 이 보여야 함.
  채널 뒤 장치 확인은 `sudo i2cset -y 1 0x70 0x01 && sudo i2cdetect -y 1` (0x01 = CH0, 0x02 = CH1, 0x04 = CH2, 0x08 = CH3).

**Pixhawk 연결** (v0.1 과 같음)

- TELEM2 <-> RPi5 GPIO UART: TX, RX 교차, GND 공통. 둘 다 3.3 V TTL. TELEM 의 5 V 핀은 연결하지 않음.
  `/boot/firmware/config.txt` 에 `dtparam=uart0=on` -> `/dev/ttyAMA0`. USB 직결이면 `/dev/ttyACM0` (`fcu_url:=` 로 지정).
- ArduPilot 파라미터: `SERIAL2_PROTOCOL=2`, `SERIAL2_BAUD=921`,
  `SR2_POSITION=10`, `SR2_EXTRA1=10`, `SR2_EXTRA2=5`, `SR2_EXT_STAT=2`, `SR2_EXTRA3=2`, `SR2_RAW_SENS=0`.
- 권한: `sudo usermod -aG dialout,i2c $USER` 후 재로그인.

## 4. 설치 (RPi5, Ubuntu 24.04 + ROS 2 Jazzy)

```bash
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras python3-serial python3-smbus2 \
                 python3-colcon-common-extensions i2c-tools
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh      # 한 번만
pip install --break-system-packages "bme680>=2.0"                     # BME688 보정 계산 (pimoroni)

cd ~/moving-ver/ros2_ws
colcon build --symlink-install
echo "source ~/moving-ver/ros2_ws/install/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

## 5. 실행

```bash
ros2 launch odor_drone fake.launch.py                       # 하드웨어 없이
ros2 launch odor_drone real.launch.py                       # 실기체: mavros + I2C 센서
ros2 launch odor_drone real.launch.py fc:=fake              # 책상에서 센서만 (위치는 가짜 원 비행)
ros2 launch odor_drone real.launch.py gas:=serial           # 아두이노 센서 모듈 (v0.1 방식)
ros2 launch odor_drone real.launch.py fcu_url:=serial:///dev/ttyACM0:115200 mavros_launch:=px4.launch
```

센서 주소, 채널, ID, 로그 폴더는 `ros2_ws/src/odor_drone/config/mob_01.yaml`.

확인:

```bash
ros2 topic hz /drone/state              # 10 Hz
ros2 topic echo --once /odor/sample
ros2 topic echo --once /health/mob_01   # sensor_ok_mask 31 = 센서 4 개 + 멀티플렉서 정상
ls ~/odor_logs/                         # 실행마다 odor_YYYYmmdd_HHMMSSZ.csv, .jsonl
python3 ~/moving-ver/tools/validate_jsonl.py ~/odor_logs/*.jsonl
```

## 6. 자주 막히는 곳

| 증상 | 확인 |
|---|---|
| `/drone/state` 위치가 null | GPS fix 가 아직 없음 (v0.2 부터 0 대신 null). `gps_fix_type` 과 위성 수 확인 |
| `/drone/state` 가 안 옴 | `ros2 topic hz /mavros/state`. 포트, 보레이트, TX/RX 교차 |
| `gas_i2c_node` 가 시작하자마자 죽음 | `/dev/i2c-1` 없음 -> I2C 를 켜고, 사용자를 i2c 그룹에 |
| `/health/mob_01` 의 mask 에서 비트가 빠짐 | 그 센서 배선. 노드가 30 초마다 다시 붙여 보고 로그에 "연결됨" 을 남김 |
| `tvoc_ppb` 가 처음 15 초 null | SGP30 초기화 중 (`VOC_INVALID`). 정상 |
| flags 에 `TIME_UNSYNCED`(64) | Pi 시계가 NTP 에 안 맞음. 현장에서는 RTC 배터리나 GPS 시각 필요 (DATA_SPEC 11장) |
| `odor_sample_node` 가 "드론 상태가 N초 전 값" | FC 링크 끊김. 샘플은 계속 나오지만 `drone_age_s` 가 커짐 |

## 7. 시험

```bash
python3 -m pytest tests ros2_ws/src/odor_drone/test        # ROS 없이 (rclpy 시험은 건너뜀)
cd ros2_ws && colcon build && source install/setup.bash && python3 -m pytest src/odor_drone/test   # ROS 포함
```

규격(.msg)을 고친 뒤에는:

```bash
python3 tools/gen_schemas.py && python3 tools/gen_docs.py && python3 tools/make_examples.py
python3 -m pytest tests
```
