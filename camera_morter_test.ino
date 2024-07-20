#include "BluetoothSerial.h"

// Bluetoothの設定
BluetoothSerial SerialBT;

// モーターのピン
const int STBY = 2;     // モータードライバの制御の準備
const int AIN1 = 26;     // 1つ目のDCモーターの制御
const int AIN2 = 27;     // 1つ目のDCモーターの制御
const int BIN1 = 25;     // 2つ目のDCモーターの制御
const int BIN2 = 33;     // 2つ目のDCモーターの制御
const int PWMA = 4;     // 1つ目のDCモーターの回転速度
const int PWMB = 32;    // 2つ目のDCモーターの回転速度
//int RX_PIN = 16;
//int TX_PIN = 17;

//カメラの設定
const int pix=320; //画素数

void setup() {
  // カメラの初期化
  Serial.begin(115200, SERIAL_8N1, 3, 1);

  // モーターの初期化
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Bluetoothの初期化
  SerialBT.begin("ESP32_naotaka"); //デバイス名
  while(!SerialBT.hasClient()) {
    delay(1000);
  }

  SerialBT.println("Bluetooth connected!");

  delay(1000);
}

void loop() {
  Camera();
  exit(0); //loopを1回で終了
}

// 前進
void Forward(int i, int j) {
  analogWrite(PWMA, i);
  analogWrite(PWMB, j);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

// 回転（0:右回転. 1:左回転）
void Turn(int a, int i, int j) {
  if(a == 0) {
    analogWrite(PWMA, i);
    analogWrite(PWMB, j);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }

  if(a == 1) {
  analogWrite(PWMA, i);
  analogWrite(PWMB, j);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  }
}

// 後退
void Back(int i, int j) {
  analogWrite(PWMA, i);
  analogWrite(PWMB, j);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

// 停止
void Stop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void Camera() {
  const double Kp = 0.01;
  int x=-1, y=0;
  float percentage;

  // データが受信されている場合
  while(1){
    if (Serial.available()) {
      // 受信データを読み取る
      String receivedData = Serial.readStringUntil('\n');
      
      // 受信データをコンマで分割
      int firstComma = receivedData.indexOf(',');
      int secondComma = receivedData.indexOf(',', firstComma + 1);
      int spaceAfterSecondComma = receivedData.indexOf(' ', secondComma + 1);
      
      // X座標
      String xStr = receivedData.substring(1, firstComma); // "R"の後から最初のカンマまで
      x = xStr.toInt();
      
      // Y座標
      String yStr = receivedData.substring(firstComma + 1, secondComma);
      y = yStr.toInt();
      
      // 割合
      String percentageStr = receivedData.substring(secondComma + 1);
      percentageStr.trim();
      percentage = percentageStr.toFloat();

      SerialBT.println(x);
      SerialBT.println(percentage);
    }
  
    // 短い遅延
    delay(10);

    if(x == -1) {
      //検出されないため、左右どちらかに旋回し続ける
      Turn(1, 50, 50);
    }
    else {
      Stop();
      int p_pwma= constrain(Kp * (x - pix) + 200, 0, 255);
      int p_pwmb = constrain(-Kp * (x - pix) + 200, 0, 255);
      Forward(p_pwma, p_pwmb);
    }

    if(percentage > 80.0){
      //ゴール判定
      Stop();
      break;
    }
  }
}