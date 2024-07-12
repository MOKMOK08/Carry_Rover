#include <Wire.h>
#include <Adafruit_BNO055.h>

// BNO055の設定
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  // BNO055の初期化
  pinMode(21, INPUT_PULLUP); // SDAピン21のプルアップ設定
  pinMode(22, INPUT_PULLUP); // SDAピン22のプルアップ設定

  if (!bno.begin()) {
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  Serial.begin(115200);

  delay(1000);
}

void loop() {
  Release();
  exit(0);
}

// クオータニオンをオイラー角に変換
double Euler(int axis) {
  imu::Quaternion quat = bno.getQuat();
  double w = quat.w();
  double x = quat.x();
  double y = quat.y();
  double z = quat.z();

  double ysqr = y * y;

  // roll (x-axis rotation)
  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + ysqr);
  double roll = atan2(t0, t1);

  // pitch (y-axis rotation)
  double t2 = +2.0 * (w * y - z * x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  double pitch = asin(t2);

  // yaw (z-axis rotation)
  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (ysqr + z * z);
  double yaw = atan2(t3, t4);

  //ラジアンから度に変換
  roll *= 57.2957795131;
  pitch *= 57.2957795131;
  yaw *= 57.2957795131;

  // 軸指定
  if(axis == 0) {
    return roll;
  }
  else if(axis == 1) {
    return pitch;
  }
  else if(axis == 2) {
    return yaw;
  }
}

//放出判定
void Release() {
  int i = 0, j = 0;
  double ave_roll = 0.0; // 平均ロール角度

  while(1) {
    for(i = 0; i < 10; i++) {
      ave_roll += fabs(Euler(0));
      delay(10);
    }
    ave_roll /= 10;

    if(ave_roll < 45) {
      j++;
    }
    else {
      j = 0;
    }

    if(j >= 5) {
      break;
    }
  }
}