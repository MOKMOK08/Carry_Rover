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
  Serial.print("φ = ");
  Serial.println(Euler(0));
  Serial.print("θ = ");
  Serial.println(Euler(1));
  Serial.print("𝜓 = ");
  Serial.println(Euler(2));
  delay(100);
}

// クオータニオンをオイラー角に変換
double Euler(int axis) {
  double roll = 0;
  double pitch = 0;
  double yaw = 0;

  imu::Quaternion quat = bno.getQuat();
  double w = quat.w();
  double x = quat.x();
  double y = quat.y();
  double z = quat.z();
  double ysqr = y * y;

  if(axis == 0) {
    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + ysqr);
    roll = atan2(t0, t1) * 57.2957795131;
    return roll;
  }
  else if(axis == 1){
    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    pitch = asin(t2) * 57.2957795131;
    return pitch;
  }
  else if(axis == 2){
    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (ysqr + z * z);  
    yaw = atan2(t3, t4) * 57.2957795131;
    return yaw;
  }
}