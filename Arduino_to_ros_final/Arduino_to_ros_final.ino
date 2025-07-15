// 최종 실전용 IMU 데이터 전송 스케치 (v2)
// 수동으로 캘리브레이션 값을 적용하는 최종 버전
// 라이브러리: MPU9250_asukiaaa

#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mySensor;

// =======================================================================
// ===> 1단계에서 정리한 최종 보정값들을 여기에 붙여넣으세요! <===
// =======================================================================
// 자이로 바이어스 (단위: dps - 도/초)
const float G_B_X_dps = -0.0128 * 180.0 / M_PI;
const float G_B_Y_dps = 0.1062 * 180.0 / M_PI;
const float G_B_Z_dps = -0.0139 * 180.0 / M_PI;

// 가속도계 바이어스 (단위: g)
const float A_B_X_g = -0.0553;
const float A_B_Y_g = 0.0071;
const float A_B_Z_g = 0.0511;

// 가속도계 스케일
const float A_S_X = -0.9979;
const float A_S_Y = -1.0015;
const float A_S_Z = -1.0145;

// 지자기 바이어스 (단위: uT)
const float M_B_X = 87.5650;
const float M_B_Y = -115.5700;
const float M_B_Z = -64.8850;
// =======================================================================


void setup() {
  Serial.begin(115200);
  while(!Serial) {}

  Wire.begin();
  mySensor.setWire(&Wire);

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  
  Serial.println("Calibrated MPU9250 is online and sending data.");
}
// 최종 완성형 loop() 함수 - dtostrf 사용

void loop() {
  // 센서 값 업데이트 (기존과 동일)
  mySensor.accelUpdate();
  mySensor.gyroUpdate();
  mySensor.magUpdate();

  // 값 계산 및 단위 변환 (기존과 동일)
  float cal_ax = (mySensor.accelX() - A_B_X_g) / A_S_X;
  float cal_ay = (mySensor.accelY() - A_B_Y_g) / A_S_Y;
  float cal_az = (mySensor.accelZ() - A_B_Z_g) / A_S_Z;
  float cal_gx_dps = mySensor.gyroX() - G_B_X_dps;
  float cal_gy_dps = mySensor.gyroY() - G_B_Y_dps;
  float cal_gz_dps = mySensor.gyroZ() - G_B_Z_dps;
  float cal_mx = mySensor.magX() - M_B_X;
  float cal_my = mySensor.magY() - M_B_Y;
  float cal_mz = mySensor.magZ() - M_B_Z;
  
  const float g_to_mss = 9.80665;
  const float dps_to_rads = M_PI / 180.0;
  
  float final_ax = cal_ax * g_to_mss;
  float final_ay = cal_ay * g_to_mss;
  float final_az = cal_az * g_to_mss;
  float final_gx_rads = cal_gx_dps * dps_to_rads;
  float final_gy_rads = cal_gy_dps * dps_to_rads;
  float final_gz_rads = cal_gz_dps * dps_to_rads;

  // ====================================================================
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
  // ====================================================================

  delay(20);
}
