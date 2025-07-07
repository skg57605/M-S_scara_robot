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

void loop() {
  // 모든 센서 값 업데이트
  mySensor.accelUpdate();
  mySensor.gyroUpdate();
  mySensor.magUpdate();

  // --- 1. 원시(raw) 센서 값 읽기 ---
  float ax, ay, az;
  float gx_dps, gy_dps, gz_dps;
  float mx, my, mz;

  ax = mySensor.accelX();
  ay = mySensor.accelY();
  az = mySensor.accelZ();

  gx_dps = mySensor.gyroX();
  gy_dps = mySensor.gyroY();
  gz_dps = mySensor.gyroZ();

  mx = mySensor.magX();
  my = mySensor.magY();
  mz = mySensor.magZ();

  // --- 2. 캘리브레이션 값 직접 적용 ---
  // 가속도: (측정값 - 바이어스) / 스케일. 단위: g
  float cal_ax = (ax - A_B_X_g) / A_S_X;
  float cal_ay = (ay - A_B_Y_g) / A_S_Y;
  float cal_az = (az - A_B_Z_g) / A_S_Z;

  // 자이로: 측정값 - 바이어스. 단위: dps
  float cal_gx_dps = gx_dps - G_B_X_dps;
  float cal_gy_dps = gy_dps - G_B_Y_dps;
  float cal_gz_dps = gz_dps - G_B_Z_dps;
  
  // 지자기: 측정값 - 바이어스. 단위: uT
  float cal_mx = mx - M_B_X;
  float cal_my = my - M_B_Y;
  float cal_mz = mz - M_B_Z;
  
  // ROS에서 사용할 단위로 변환
  const float g_to_mss = 9.80665;
  const float dps_to_rads = M_PI / 180.0;
  
  float final_ax = cal_ax * g_to_mss;
  float final_ay = cal_ay * g_to_mss;
  float final_az = cal_az * g_to_mss;

  float final_gx_rads = cal_gx_dps * dps_to_rads;
  float final_gy_rads = cal_gy_dps * dps_to_rads;
  float final_gz_rads = cal_gz_dps * dps_to_rads;


  // PC로 전송할 데이터 포맷: "IMU:ax,ay,az,gx,gy,gz,mx,my,mz"
  Serial.print("IMU:");
  Serial.print(final_ax, 4); Serial.print(",");
  Serial.print(final_ay, 4); Serial.print(",");
  Serial.print(final_az, 4); Serial.print(",");
  Serial.print(final_gx_rads, 4); Serial.print(","); // rad/s 단위로 전송
  Serial.print(final_gy_rads, 4); Serial.print(",");
  Serial.print(final_gz_rads, 4); Serial.print(",");
  Serial.print(cal_mx, 4); Serial.print(","); // uT 단위로 전송
  Serial.print(cal_my, 4); Serial.print(",");
  Serial.println(cal_mz, 4);

  delay(20); // 약 50Hz로 데이터 전송
}
