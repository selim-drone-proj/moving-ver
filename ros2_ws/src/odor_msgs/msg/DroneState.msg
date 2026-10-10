# 드론 비행 컨트롤러(Pixhawk) 상태 요약
# fc_state_node 가 mavros 토픽들을 모아서 /drone/state 로 10 Hz 발행한다.

std_msgs/Header header      # stamp: 발행 시각(RPi 시계), frame_id: 드론 ID

# ---- 상태 ----
bool   armed
string mode                 # 비행모드 문자열 (예: LOITER, AUTO, GUIDED, RTL)
bool   in_air               # 공중이면 true, 지상이면 false

# ---- 위치 (FC 내부 EKF 융합값). 아직 모르거나 GPS fix 가 없으면 NaN (v0.2) ----
float64 latitude            # deg, WGS84. 모르면 NaN
float64 longitude           # deg. 모르면 NaN
float32 altitude_msl        # m, 해수면 기준
float32 altitude_rel        # m, 이륙 지점 기준
float32 heading_deg         # 0~360, 북쪽이 0, 시계방향 +
uint8   gps_fix_type        # 0~1: fix 없음, 2: 2D, 3: 3D, 4~6: DGPS/RTK
uint8   gps_satellites      # 사용 위성 수

# ---- 자세, 속도 ----
float32 roll_deg
float32 pitch_deg
float32 yaw_deg             # ENU 기준(동쪽이 0, 반시계 +). 방위는 heading_deg 를 쓸 것
float32 ground_speed_mps    # m/s
float32 climb_rate_mps      # m/s, 상승이 +

# ---- 배터리 (FC 파워모듈 기준) ----
float32 battery_voltage     # V
float32 battery_current     # A, 방전 중일 때 양수
float32 battery_percent     # 0~100, FC 추정치. 모르면 -1
