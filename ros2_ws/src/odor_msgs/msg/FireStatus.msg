# 화재 감지기(ess_01) 상태. fire_detect 가 /fire/<id>/status 로 1 Hz 발행한다.
# 감시 대상: 소형 ESS(파워뱅크) 충전 중 과열. 센서: 표면온도(AMG8833 열화상, 보조로 DS18B20 접촉 프로브와
# MLX90614 적외선), 주변 온습도(SHT31), 연기(MQ-2). 접촉 프로브, 적외선 값은 이 메시지에 없고(감지기 CSV 기록)
# 판정(level)과 flags 에만 들어간다

std_msgs/Header header      # stamp: 판정 시각(UTC), frame_id: 장비 ID (ess_01)

uint8 LEVEL_NORMAL  = 0     # 정상: 주변보다 뜨거운 화소도, 뜨거운 점 센서도 없음. 녹색 LED
uint8 LEVEL_CAUTION = 1     # 주의: 뜨거운 화소 1~2개 (국부 발열), 또는 접촉 프로브나 적외선이 뜨거움. 황색 LED
uint8 LEVEL_WARNING = 2     # 경고: 뜨거운 화소 3개 이상 (번지는 중). 적색 점멸, 부저 간헐
uint8 LEVEL_DANGER  = 3     # 위험: 연기, 또는 화소나 점 센서가 절대 한계 초과. 적색 상시, 부저 연속, 충전 차단

uint8   level               # 위 LEVEL_* 4 단계 (0 정상 ~ 3 위험)
bool    latched             # 위험 단계가 걸려 사람이 해제할 때까지 유지되는 중

float32 ambient_temp_c      # 주변 온도 ℃ (SHT31). 뜨거운 화소 판정의 기준선
float32 ambient_rh_pct      # 주변 상대습도 %
float32 surface_max_c       # 표면 64 화소 중 최고 ℃
float32 surface_mean_c      # 표면 64 화소 평균 ℃
float32 surface_min_c       # 표면 64 화소 중 최저 ℃
uint8   hot_pixels          # (화소 온도 > 주변 + delta_c) 인 화소 개수 0~64
uint8   hotspot_index       # 최고 화소 위치 0~63 = 행 * 8 + 열. 255 = 없음
float32 delta_c             # 판정에 쓴 ΔT ℃ (설정값)
float32 abs_limit_c         # 판정에 쓴 절대 한계 ℃ (설정값)

float32 smoke_v             # MQ-2 출력 전압 V (분압 보정 후 실제 AO 전압)
float32 smoke_baseline_v    # 깨끗한 공기 기준 전압 V (정상 단계에서 천천히 학습)
bool    smoke_detected      # smoke_v 가 기준보다 설정값 이상 올라감

bool    charge_allowed      # AC SSR 통전 중 = 충전 허용. false = 차단
bool    dc_allowed          # DC 릴레이 통전 중
bool    buzzer_on           # 부저가 울리는 중 (연속 또는 간헐)

uint16  flags               # 아래 비트. bit6, bit11 은 GasReading 과 같은 뜻
float32[64] thermal_c       # 8x8 화소 온도 ℃, 행 우선 (0 번이 왼쪽 위)

uint16 FLAG_SMOKE_WARMUP  = 1      # bit0  MQ-2 예열 중이라 연기 판정을 끔
uint16 FLAG_AMG_INVALID   = 2      # bit1  열화상 읽기 실패
uint16 FLAG_SHT_INVALID   = 4      # bit2  온습도 읽기 실패 (주변 온도 대신 화소 최저값 사용)
uint16 FLAG_ADC_INVALID   = 8      # bit3  연기 ADC 읽기 실패
uint16 FLAG_PROBE_INVALID = 16     # bit4  접촉 프로브(DS18B20) 중 못 읽은 것이 있음
uint16 FLAG_IR_INVALID    = 32     # bit5  적외선 온도계(MLX90614) 읽기 실패
uint16 FLAG_TIME_UNSYNCED = 64     # bit6  시각을 믿을 수 없음
uint16 FLAG_SIMULATED     = 2048   # bit11 가짜 데이터
