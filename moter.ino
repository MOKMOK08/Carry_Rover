const int STBY = 0;     // モータードライバの制御の準備
const int AIN1 = 26;     // 1つ目のDCモーターの制御
const int AIN2 = 27;     // 1つ目のDCモーターの制御
const int BIN1 = 25;     // 2つ目のDCモーターの制御
const int BIN2 = 33;     // 2つ目のDCモーターの制御
const int PWMA = 4;     // 1つ目のDCモーターの回転速度
const int PWMB = 32;    // 2つ目のDCモーターの回転速度
int morter = 0;

void setup() {
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  digitalWrite(STBY, HIGH);
}

void loop() {
  Forward(255, 255);
  delay(500);
  Turn(0, 100, 100);
  delay(500);
  Forward(255, 255);
  delay(500);
  Stop();
  while(1) {
  }
}

// 前進
void Forward(int i, int j) {
  morter = 1;
  analogWrite(PWMA, i);
  analogWrite(PWMB, j);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

// 回転（0:右回転. 1:左回転）
void Turn(int a, int i, int j) {
  if(morter == 1) {
    Gensoku();
  }
  morter = 2;
  if(a == 0) {
    analogWrite(PWMA, i);
    analogWrite(PWMB, j);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }

  if(a == 1) {
  analogWrite(PWMA, i);
  analogWrite(PWMB, j);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
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

// 減速
void Gensoku() {
  for(int i = 200; i > 0; i -= 10) {
    Forward(i, i);
    delay(40);
  }
}

// 停止
void Stop() {
  if(morter == 1) {
    Gensoku();
  }
  morter = 0;
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}
