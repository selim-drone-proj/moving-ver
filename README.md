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
| `lora_uplink_node` | 최신 샘플을 2 초마다 LoRa 요약 패킷(type 0x03)으로 E220 에 씀 (`uplink:=lora`) | `/odor/sample` | (E220 -> 지상국) |
| `fake_fc_node`, `fake_gas_node` | 하드웨어 없이 같은 토픽을 가짜로 | | `/drone/state`, `/odor/gas` |

v0.1 은 ROS 기능을 pub/sub 와 타이머만 썼습니다. v0.2 는 여기에 **파라미터**만 더해 장비 ID, 센서 주소를
`config/mob_01.yaml` 에서 바꿀 수 있게 했습니다. 서비스, 액션은 여전히 쓰지 않습니다.

## 3. 센서 배선 (mob_01)

실물 배선(2026-11-20 확인) 기준. 결선 도면(2026-10-05)의 CH0~3 배치와 다릅니다. 센서는 드론 붐에 노출 장착하고, 전자부만 방수 함체에 넣습니다.

| TCA9548A 채널 | 센서 | 주소 | 전원 (Pi5 40핀) | 채우는 필드 |
|---|---|---|---|---|
| CH5 | Gravity NH3 SEN0469 | 0x74 | **2번 5 V**, GND 14번 | nh3_ppm, nh3_raw |
| CH6 | Gravity H2S SEN0467 | 0x74 | 17번 3.3 V, GND 25번 | h2s_ppm, h2s_raw |
| CH3 | Grove SGP30 | 0x58 | | tvoc_ppb, eco2_ppm |
| CH2 | Gravity BME688 | 0x77 (안 되면 0x76 자동) | | temperature_c, humidity_pct, pressure_hpa, gas_resistance_ohm |

- TCA9548A (0x70) 는 Pi5 40핀 1번(3.3 V), 3번(SDA), 5번(SCL), 6번(GND).
- **NH3 보드는 5 V 로 켭니다.** 3.3 V 로 켜면 BME688 가스 히터가 도는 순간 NH3 가 빈 응답(가스 종류 0x00)을 내고
  "NH3 끊김" 이 됩니다. 두 Gravity 의 VCC 를 맞바꿔 확인했고, H2S 는 3.3 V 에서도 정상입니다.
  Gravity 의 SDA, SCL 은 반드시 TCA9548A 채널에 둡니다 (5 V 로 켠 보드의 풀업이 Pi 3.3 V 핀에 직접 걸리지 않게).
- **I2C 는 10 kHz 로 낮춥니다.** `/boot/firmware/config.txt` 에 `dtparam=i2c_arm_baudrate=10000`.
  100 kHz 에서는 Gravity 보드가 클록 스트레칭 중 SCL 을 붙잡고 멈춰 버스 전체가 막힙니다
  (`Errno 110`, 커널 `i2c_designware ... timeout waiting for bus ready`). 이 상태는 센서 전원을 끊어야 풀립니다.
- Gravity 두 개가 같은 0x74 라서 멀티플렉서로 나눕니다. 보드 뒷면 딥스위치로 주소를 다르게 하면
  멀티플렉서 없이 직결해도 되고, 그때는 `tca_address: -1` (BME688 이 0x76, 0x77 을 쓰므로 Gravity 는 0x74, 0x75 로).
- 채널을 바꿔 꽂으면 노드가 "NH3 자리에 H2S 센서가 연결됨" 이라고 알려 줍니다. 채널은 `config/mob_01.yaml` 의 `*_channel`.
- I2C 켜기: `/boot/firmware/config.txt` 에 `dtparam=i2c_arm=on`, 재부팅 후 `sudo i2cdetect -y 1` 에 `70` 이 보여야 함.
  채널 뒤 장치 확인은 `sudo i2cset -y 1 0x70 <비트> && sudo i2cdetect -y 1` (0x04 = CH2, 0x08 = CH3, 0x20 = CH5, 0x40 = CH6).

**Pixhawk 연결** (v0.1 과 같음)

- TELEM2 <-> RPi5 GPIO UART: TX, RX 교차, GND 공통. 둘 다 3.3 V TTL. TELEM 의 5 V 핀은 연결하지 않음.
  `/boot/firmware/config.txt` 에 `dtparam=uart0=on` -> `/dev/ttyAMA0`. USB 직결이면 `/dev/ttyACM0` (`fcu_url:=` 로 지정).
- ArduPilot 파라미터: `SERIAL2_PROTOCOL=2`, `SERIAL2_BAUD=921`,
  `SR2_POSITION=10`, `SR2_EXTRA1=10`, `SR2_EXTRA2=5`, `SR2_EXT_STAT=2`, `SR2_EXTRA3=2`, `SR2_RAW_SENS=0`.
- 권한: `sudo usermod -aG dialout,i2c $USER` 후 재로그인.

**E220-900T22D 연결** (`uplink:=lora`. 지상국은 stable-ver `odor-gateway` 가 받음)

`uart0` 은 Pixhawk 가 쓰므로 E220 은 **Pi5 uart2 에 직결**합니다 -> `/dev/ttyAMA2` (2026-11-20 실물 확인).
E220 의 신호선은 3.3 V 라 레벨 컨버터가 필요 없습니다. USB-TTL(CP2102) 로 붙이면 `/dev/ttyUSB0` 이고 `port` 만 바꾸면 됩니다.

| E220 핀 | Pi5 40핀 | 비고 |
|---|---|---|
| VCC | 4번 (5 V) | |
| GND | 9번 (GND) | |
| TXD | 29번 (GPIO5, uart2 RX) | 교차 |
| RXD | 7번 (GPIO4, uart2 TX) | 교차 |
| M0 | 16번 (GPIO23) | `tools/e220.py` 가 설정 모드 때만 HIGH 로 올림 |
| M1 | 18번 (GPIO24) | 〃 |
| AUX | 연결 안 함 | |

- `/boot/firmware/config.txt` 에 두 줄을 넣고 재부팅합니다.
  ```
  dtoverlay=uart2-pi5          # GPIO4/5 -> /dev/ttyAMA2 (uart3 은 GPIO8 이 SPI CE0 와 겹침)
  gpio=23,24=op,dl             # 부팅 때 M0 = M1 = LOW -> 투명 모드
  ```
- 무선 설정: 채널 72 (922.125 MHz), 주소 0, 공중 속도 2.4 kbps, UART 9600 8N1, 22 dBm, 투명 전송.
  공장 기본값은 채널 18 (868.125 MHz, 유럽 대역)이라 **반드시 바꿉니다**. 주파수 = 850.125 + 채널 MHz.
  `LORA_PACKET.md` 1장 권고대로 고정형 관측소와 채널을 나눴습니다 (고정형 70 가정). 지상국의 이동형 수신 모듈도 같게 맞춥니다.
- `tools/e220.py` (M0/M1 을 GPIO 로 자동 전환, `sudo gpioset` 필요 -> `sudo apt install gpiod`):
  ```bash
  python3 tools/e220.py read        # 설정 읽기
  python3 tools/e220.py setch 72    # 채널 영구 저장
  python3 tools/e220.py listen      # 투명 모드 수신 hex 출력
  ```
- 확인: `ros2 launch odor_drone real.launch.py uplink:=lora` 뒤 지상국에서 `odor-gateway --port /dev/ttyUSB0 --out ~/odor_logs`
  에 `mob_01 seq=...` 줄이 2 초마다 나오는지 봅니다. (2026-11-20: Pi 쪽 송신 29 건 / 오류 0 확인, 지상국 수신은 미확인)
- E220 이 아무 응답도 없을 때: 전원이 들어간 E220 의 TXD 는 쉴 때 HIGH 입니다.
  `sudo gpioget -B pull-down gpiochip4 5` 가 0 이면 E220 이 꺼졌거나 TXD 가 29번에 없습니다
  (gpioget 은 핀을 UART 에서 빼앗으므로 확인 뒤 재부팅).

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
ros2 launch odor_drone real.launch.py uplink:=lora              # E220 으로 지상국에 요약 송신
ros2 launch odor_drone real.launch.py fc:=fake gas:=fake uplink:=lora   # 책상에서 LoRa 링크만 시험
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
| 시작 몇 초 뒤 "NH3 끊김으로 판단" | NH3 VCC 가 3.3 V 임 -> 5 V 로 (3장) |
| I2C 전체가 `Errno 110`, `i2cdetect` 에서 `70` 도 안 보임 | Gravity 가 버스를 붙잡음. I2C 10 kHz 설정 확인, 어댑터를 뽑아 센서 전원을 끊었다 켬 |
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
