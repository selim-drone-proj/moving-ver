# 지오태깅된 악취 샘플 = 가스 측정값 + 그 순간의 드론 상태 (이동형)
# odor_sample_node 가 /odor/sample 로 발행한다 (가스 측정 1회당 1개, 1 Hz).
# 소프트웨어(모델, 대시보드)는 이동형 데이터를 이 토픽 하나로 받는다.
# 규격 v0.2: 맨 뒤에 drone_age_s 추가.

std_msgs/Header header      # stamp: 가스 측정 시각(UTC), frame_id: 드론 ID (octo_01)

DroneState drone            # 측정 순간의 드론 위치, 자세, 배터리
GasReading gas              # 가스 측정값

float32 drone_age_s         # 붙인 드론 상태가 몇 초 전 것인지. 2 초를 넘으면 위치를 믿지 말 것
