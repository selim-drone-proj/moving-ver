# 악취 센서 1회 측정값 (이동형, 고정형 공통)
#  - 이동형: gas_i2c_node 가 /odor/gas 로 1 Hz 발행 (순간값)
#  - 고정형: StationSample.gas 안에 들어감 (avg_window_s 동안의 평균)
# 규격 v0.2. v0.1 필드는 이름, 순서 그대로이고 뒤에 필드만 추가했다.
# 값이 없거나 그 장비에 없는 센서는 NaN (JSON 에서는 null, CSV 에서는 빈 칸).

std_msgs/Header header      # stamp: 측정 시각(UTC), frame_id: 장비 ID (mob_01, fix_01 ...)

# ---- v0.1 ----
float32 h2s_ppm             # 황화수소 ppm
float32 nh3_ppm             # 암모니아 ppm
float32 tvoc_ppb            # 총휘발성유기화합물 ppb (SGP30. SGP41 은 이 값을 못 냄 -> NaN)
float32 temperature_c       # 측정 공기 온도 ℃
float32 humidity_pct        # 측정 공기 상대습도 0~100 %

bool    pump_on             # 흡입 펌프 동작 여부 (펌프 없는 장비는 false)
float32 payload_voltage     # 센서 모듈 전원 전압 V (측정 안 하면 NaN)

# ---- v0.2 추가 ----
float32 eco2_ppm            # SGP30 eCO2 추정값 ppm (실제 CO2 센서 아님)
float32 voc_index           # SGP41 VOC Index 1~500, 평소 100. 단위 없음
float32 nox_index           # SGP41 NOx Index 1~500, 평소 1. 단위 없음
float32 pressure_hpa        # 기압 hPa (BME688)
float32 gas_resistance_ohm  # BME688 가스 저항 Ω (낮을수록 VOC 많음)
float32 pm1_0_ugm3          # PMS7003 PM1.0 µg/m³ (대기 환산값)
float32 pm2_5_ugm3          # PMS7003 PM2.5 µg/m³
float32 pm10_ugm3           # PMS7003 PM10 µg/m³
float32 nh3_raw             # NH3 원신호 (재보정용, 분석에는 쓰지 말 것). 고정형: 증폭 회로 출력 µV, 이동형: 온도 보정 전 ppm
float32 h2s_raw             # H2S 원신호 (재보정용, 분석에는 쓰지 말 것). 고정형: 증폭 회로 출력 µV, 이동형: 온도 보정 전 ppm

uint16  flags               # 품질 비트 (아래 상수). 0 이면 모두 정상

uint16 FLAG_WARMUP        = 1      # bit0  센서 예열 중
uint16 FLAG_NH3_INVALID   = 2      # bit1  암모니아 값 무효
uint16 FLAG_H2S_INVALID   = 4      # bit2  황화수소 값 무효
uint16 FLAG_VOC_INVALID   = 8      # bit3  VOC, TVOC, eCO2 무효 (SGP 초기화, 학습 중)
uint16 FLAG_TH_INVALID    = 16     # bit4  온습도 무효
uint16 FLAG_PM_INVALID    = 32     # bit5  먼지 무효
uint16 FLAG_TIME_UNSYNCED = 64     # bit6  측정 시각을 믿을 수 없음 (RTC 미설정, NTP 미동기)
uint16 FLAG_ZERO_CAL      = 128    # bit7  영점 보정 중. 외기 측정값이 아님
uint16 FLAG_PURGE         = 256    # bit8  챔버 퍼지 중. 외기 측정값이 아님
uint16 FLAG_PUMP_ON       = 512    # bit9  펌프 동작 (pump_on 과 같은 값)
uint16 FLAG_TEMP_COMP     = 1024   # bit10 가스 값에 온도 보정 적용됨 (하나라도. 원값은 *_raw)
uint16 FLAG_SIMULATED     = 2048   # bit11 가짜 데이터 (fake 노드, 시뮬레이터)
