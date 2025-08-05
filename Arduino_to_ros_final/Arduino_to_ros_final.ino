// 최종 실전용 IMU 데이터 전송 스케치 (v2)
// 수동으로 캘리브레이션 값을 적용하는 최종 버전
// 라이브러리: MPU9250_asukiaaa

#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mySensor;

void setup() {
  Serial.begin(115200);
  while(!Serial) {}

  Wire.begin();
  mySensor.setWire(&Wire);

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  Serial.println("acc_ax, acc_ay, acc_az");
  //Serial.println("Calibrated MPU9250 is online and sending data.");
}
// 최종 완성형 loop() 함수 - dtostrf 사용

void loop() {
  // 센서 값 업데이트 (기존과 동일)
  mySensor.accelUpdate();
  mySensor.gyroUpdate();
  mySensor.magUpdate();

  // 값 계산 및 단위 변환 (기존과 동일)
  float cal_ax = mySensor.accelX();
  float cal_ay = mySensor.accelY();
  float cal_az = mySensor.accelZ();
  float cal_gx_dps = mySensor.gyroX();
  float cal_gy_dps = mySensor.gyroY();
  float cal_gz_dps = mySensor.gyroZ();
  float cal_mx = mySensor.magX();
  float cal_my = mySensor.magY();
  float cal_mz = mySensor.magZ();
  
  const float g_to_mss = 9.80665;
  const float dps_to_rads = M_PI / 180.0;
  
  float final_ax = (cal_ax * g_to_mss)+0.4;
  float final_ay = (cal_ay * g_to_mss)+0.01;
  float final_az = (cal_az * g_to_mss)-0.4;
  float final_gx_rads = cal_gx_dps * dps_to_rads;
  float final_gy_rads = cal_gy_dps * dps_to_rads;
  float final_gz_rads = cal_gz_dps * dps_to_rads;
  //Serial.print("acc_ax:");
  Serial.print(cal_ax);
  Serial.print(", ");
  //Serial.print("acc_ay:");
  Serial.print(cal_ay);
  Serial.print(", ");
  //Serial.print("acc_az:");
  Serial.println(cal_az);

  /*// ====================================================================
  // ===> 핵심 수정 사항: dtostrf()를 사용해 데이터를 문자열로 변환 <===
  // ====================================================================
  
  // 1. 데이터를 담을 충분한 크기의 문자 버퍼들을 준비합니다.
  char ax_str[10], ay_str[10], az_str[10];
  char gx_str[10], gy_str[10], gz_str[10];
  char mx_str[10], my_str[10], mz_str[10];
  char tx_buffer[200]; // 최종 문장을 담을 버퍼

  // 2. dtostrf()를 사용해 각 float 값을 문자열로 변환합니다.
  // dtostrf(float값, 전체자리수, 소수점이하자리수, 저장할버퍼);
  dtostrf(final_ax, 4, 4, ax_str);
  dtostrf(final_ay, 4, 4, ay_str);
  dtostrf(final_az, 4, 4, az_str);
  dtostrf(final_gx_rads, 4, 4, gx_str);
  dtostrf(final_gy_rads, 4, 4, gy_str);
  dtostrf(final_gz_rads, 4, 4, gz_str);
  dtostrf(cal_mx, 4, 4, mx_str);
  dtostrf(cal_my, 4, 4, my_str);
  dtostrf(cal_mz, 4, 4, mz_str);

  // 3. sprintf()를 사용해 변환된 문자열들을 하나로 합칩니다.
  //    이제 %s (문자열 형식 지정자)를 사용하므로 float 문제가 없습니다.
  sprintf(tx_buffer, "IMU:%s,%s,%s,%s,%s,%s,%s,%s,%s",
    ax_str, ay_str, az_str,
    gx_str, gy_str, gz_str,
    mx_str, my_str, mz_str
  );

  // 4. 완성된 문장을 한 번에 전송합니다.
  Serial.println(tx_buffer);
  // ====================================================================*/

  delay(20);
}
