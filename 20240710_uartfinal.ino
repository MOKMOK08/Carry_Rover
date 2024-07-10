#include <HardwareSerial.h>

// UART1を使用
HardwareSerial MySerial(1);

void setup() {
  // シリアルモニタ用のシリアル通信の開始
  Serial.begin(115200);
  
  // UART1の初期化
  MySerial.begin(115200, SERIAL_8N1, 16, 17); // RXピン=16, TXピン=17

  // 初期化メッセージ
  Serial.println("UART通信開始");
}

void loop() {
  // データが受信されている場合
  if (MySerial.available()) {
    // 受信データを読み取る
    String receivedData = MySerial.readStringUntil('\n');
    
    // シリアルモニタに表示
    Serial.println("受信データ: " + receivedData);
    
    // データをパースして整数に変換
    parseData(receivedData);
  }
  
  // 短い遅延
  delay(10);
}

void parseData(String data) {
  // 受信データをコンマで分割
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);
  int spaceAfterSecondComma = data.indexOf(' ', secondComma + 1);
  
  if (firstComma > 0 && secondComma > 0 && spaceAfterSecondComma > 0) {
    // X座標
    String xStr = data.substring(1, firstComma); // "R"の後から最初のカンマまで
    int x = xStr.toInt();
    
    // Y座標
    String yStr = data.substring(firstComma + 1, secondComma);
    int y = yStr.toInt();
    
    // 割合
    String percentageStr = data.substring(secondComma + 1, spaceAfterSecondComma);
    float percentage = percentageStr.toFloat();
    int percentageInt = (int)(percentage * 100); // 小数点以下を整数に変換
    
    // 解析結果を表示
    Serial.print("X座標: ");
    Serial.println(x);
    Serial.print("Y座標: ");
    Serial.println(y);
    Serial.print("割合: ");
    Serial.println(percentageInt);
  } else {
    Serial.println("データの解析に失敗しました");
  }
}
