/*
 * gas_module.ino  -  드론 장착 악취 센서 모듈 (MCU 쪽)
 *
 * 1초마다 시리얼로 한 줄 보낸다. RPi5 의 gas_sensor_node 가 이 줄을 읽는다.
 *   h2s_ppm,nh3_ppm,tvoc_ppb,temp_c,hum_pct,pump_on,payload_v\n
 *
 * 아래 read_* 함수는 자리만 잡아둔 것. 실제 센서(ADS1115, SGP41, SHT31 등) 라이브러리로 채울 것.
 * 단위는 ppm / ppm / ppb / 섭씨 / % / 0,1 / V 로 맞춰서 보낼 것 (RPi 쪽은 변환 안 함).
 */

const int PIN_PUMP = 7;            // 펌프 릴레이(또는 MOSFET) 제어 핀
const int PIN_VBAT = A0;           // 페이로드 배터리 전압 분배 입력
const float VBAT_DIVIDER = 11.0;   // (R1+R2)/R2, 예: 100k+10k -> 11
const unsigned long PERIOD_MS = 1000;

unsigned long last = 0;

float read_h2s_ppm()  { return 0.0; }   // TODO: 전기화학 H2S 센서
float read_nh3_ppm()  { return 0.0; }   // TODO: 전기화학 NH3 센서
float read_tvoc_ppb() { return 0.0; }   // TODO: SGP41 / AGS02MA
float read_temp_c()   { return 0.0; }   // TODO: SHT31 / BME280
float read_hum_pct()  { return 0.0; }   // TODO: SHT31 / BME280

float read_vbat() {
  return analogRead(PIN_VBAT) * (5.0 / 1023.0) * VBAT_DIVIDER;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PUMP, OUTPUT);
  digitalWrite(PIN_PUMP, HIGH);    // 시작하자마자 펌프 ON (비행 중 상시 흡입)
}

void loop() {
  if (millis() - last < PERIOD_MS) return;
  last = millis();

  int pump_on = digitalRead(PIN_PUMP) == HIGH ? 1 : 0;

  Serial.print(read_h2s_ppm(), 3);  Serial.print(',');
  Serial.print(read_nh3_ppm(), 3);  Serial.print(',');
  Serial.print(read_tvoc_ppb(), 0); Serial.print(',');
  Serial.print(read_temp_c(), 1);   Serial.print(',');
  Serial.print(read_hum_pct(), 1);  Serial.print(',');
  Serial.print(pump_on);            Serial.print(',');
  Serial.println(read_vbat(), 2);
}
