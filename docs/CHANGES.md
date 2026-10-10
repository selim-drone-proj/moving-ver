# 변경 내역 (소프트웨어)

규격(메시지, JSON, CSV) 변경은 [DATA_SPEC.md 10장](DATA_SPEC.md#10-변경-내역). 여기는 노드와 도구의 변경입니다.

## v0.2.0 (2026-10-09)

**odor_msgs**
- GasReading 에 필드 10 개 + flags, OdorSample 에 drone_age_s 추가 (맨 뒤에만)
- StationSample, FireStatus, DeviceHealth 추가

**odor_drone**
- `gas_i2c_node` 추가: Gravity NH3, H2S (DFRobot I2C 규약을 직접 구현), SGP30, BME688 을 TCA9548A 로 읽음
  - 센서 하나가 빠져도 계속 발행, 3 번 연속 실패하면 끊긴 것으로 보고 30 초마다 다시 연결
  - 예열(`WARMUP`), SGP30 초기 15 초(`VOC_INVALID`), 시각 미동기(`TIME_UNSYNCED`) 표시
  - `/health/mob_01` 로 센서 상태, CPU 온도, 5 V 입력 전압 보고
  - `simulate_bus:=true` 로 가짜 I2C 장치 위에서 실제 드라이버 코드를 돌려 볼 수 있음 (개발용)
- 드라이버 `odor_drone/drivers/`: i2c_bus, tca9548a, dfrobot_gas, sgp30, bme688, sensirion, fakes
- 모든 노드: 장비 ID 와 포트 등을 파라미터로 (기본 `octo_01`, `mob_01`), `config/mob_01.yaml`
- `fc_state_node`: 위치를 모르면 NaN (GPS fix 없음일 때 0, 0 이 기록되던 문제)
- `odor_sample_node`: `drone_age_s` 계산, FC 상태가 없어도 가스 샘플을 버리지 않음
- `csv_logger_node`: UTC 시각, 빈 칸, 열 13 개 추가, JSONL 동시 기록 (`write_jsonl`)
- `fake_gas_node`: v0.2 필드, `SIMULATED` 비트, `/health/mob_01`
- `real.launch.py`: `fc:=mavros|fake`, `gas:=i2c|serial|fake`, `fcu_url:=`, `mavros_launch:=`, `params:=`
- 종료할 때 Ctrl-C 가 두 번 들어와도 traceback 이 나지 않게

**도구**
- `tools/gen_schemas.py` (.msg -> JSON Schema), `tools/gen_docs.py` (.msg -> 문서 표)
- `tools/validate_jsonl.py`, `tools/fake_jsonl.py`, `tools/make_examples.py`
- `interfaces/python/odor_json.py`: JSON <-> ROS 변환. float32 를 짧은 십진수로 (24.01)

**시험**
- 규격 검사 34 개, 드라이버와 판독 로직 24 개, 실제 rclpy 로 하는 시험 8 개
  (ROS 2 Jazzy 에서 colcon 빌드, fake.launch.py 와 real.launch.py fc:=fake simulate_bus:=true 로 끝까지 확인)

## v0.1.0

YoungHyun1884/drone_stink 초기 커밋: fc_state_node, gas_sensor_node, odor_sample_node, csv_logger_node, fake 노드 2 개.
