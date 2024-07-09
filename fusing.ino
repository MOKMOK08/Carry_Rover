const int FUSE_GPIO = 2;

void setup() {
  pinMode(FUSE_GPIO, OUTPUT);
  digitalWrite(FUSE_GPIO, LOW);
}

void loop() {
}

void Fusing() {
  digitalWrite(FUSE_GPIO, HIGH);
  delay(500);
  digitalWrite(FUSE_GPIO, LOW);
}