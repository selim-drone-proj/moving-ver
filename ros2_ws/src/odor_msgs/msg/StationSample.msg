# 고정형 관측소 1회 보고 (fix_01 ~ fix_04)
# 관측소(Mega2560)가 1 초마다 재서 avg_window_s 동안 평균한 값을 LoRa 로 보내고,
# 지상국 게이트웨이(lora_gateway)가 받아서 /station/<id>/sample 로 발행한다.
# 이동형의 OdorSample 과 짝을 이룬다: 위치가 드론 상태 대신 고정 설치 좌표.

std_msgs/Header header      # stamp: 평균 창의 끝 시각(관측소 RTC, UTC), frame_id: 관측소 ID (fix_01)

uint8 MODE_DIFFUSION = 0    # 자연 확산 (통기구)
uint8 MODE_CHAMBER   = 1    # 챔버 + 펌프 흡입

float64 latitude            # deg, WGS84. 설치 좌표 (게이트웨이 stations.yaml)
float64 longitude           # deg
float32 altitude_msl        # m

uint8   sampling_mode       # MODE_DIFFUSION 또는 MODE_CHAMBER
uint16  avg_window_s        # 몇 초 동안의 평균인지 (보통 60)

GasReading gas              # 측정값. gas.header 는 위 header 와 같다. 품질은 gas.flags

uint16  seq                 # 관측소 패킷 순번. 건너뛰면 그만큼 손실
int16   rssi_dbm            # 게이트웨이 수신 세기 dBm. 0 = 모름
builtin_interfaces/Time received   # 게이트웨이가 받은 시각(UTC)
