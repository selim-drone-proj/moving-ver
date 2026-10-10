# 장비 상태(하트비트). 모든 장비가 /health/<장비 ID> 로 발행한다.
#  - Pi5 장비(mob_01, ess_01): 10 초마다
#  - 고정형(fix_xx): 10 분마다 LoRa 로 보내고 게이트웨이가 대신 발행
#  - 게이트웨이(gw_01): 10 초마다
# 대시보드는 이 토픽으로 "살아 있나, 센서가 다 붙어 있나" 를 본다.

std_msgs/Header header      # stamp: 보고 시각(UTC), frame_id: 장비 ID

uint8 STATE_BOOT     = 0    # 막 켜짐
uint8 STATE_WARMUP   = 1    # 센서 예열 중
uint8 STATE_OK       = 2    # 정상
uint8 STATE_DEGRADED = 3    # 일부 센서 실패, 나머지는 동작
uint8 STATE_ERROR    = 4    # 측정 불가

uint8   state               # 위 STATE_* (0 막 켜짐 ~ 4 측정 불가)
string  firmware            # 소프트웨어 이름과 버전 (예: fix_station 0.2.0)
uint32  uptime_s            # 켜진 뒤 경과 초
uint16  sensor_ok_mask      # 센서별 정상 비트. 비트 뜻은 장비마다 다름 (DATA_SPEC.md 4.6 표)
uint16  error_count         # 켜진 뒤 누적 오류 수
float32 supply_v            # 전원 전압 V. 모르면 NaN
float32 cpu_temp_c          # CPU 온도 ℃ (Pi 만). 모르면 NaN
int16   rssi_dbm            # LoRa 수신 세기 dBm (게이트웨이가 채움). 0 = 모름
string  message             # 최근 오류를 사람이 읽을 수 있게. 없으면 빈 문자열
