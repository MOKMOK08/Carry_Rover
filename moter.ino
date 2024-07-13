const int STBY = 2;     // モータードライバの制御の準備
const int AIN1 = 26;     // 1つ目のDCモーターの制御
const int AIN2 = 27;     // 1つ目のDCモーターの制御
const int BIN1 = 25;     // 2つ目のDCモーターの制御
const int BIN2 = 33;     // 2つ目のDCモーターの制御
const int PWMA = 4;     // 1つ目のDCモーターの回転速度
const int PWMB = 32;    // 2つ目のDCモーターの回転速度

void setup() {
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  digitalWrite(STBY, HIGH); // スタンバイ
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
}
yy7u7u6yyttrrtr
void loop() {
}

void Foward(int i, int j) {
  analogWrite(PWMA, i);
  analogWrite(PWMB, j);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  delay(1000);
}

void Turn(char a, int i, int j) {
  if(a = r) {
    analogWrite(PWMA, i);
    analogWrite(PWMB, j);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }

  if(a = l) {
  analogWrite(PWMA, i);
  analogWrite(PWMB, j);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  }
}

void Back(int i, int j) {
  analogWrite(PWMA, i);
  analogWrite(PWMB, j);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void Stop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}