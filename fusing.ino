const int FUSE_GPIO = 2;

void setup() {
  pinMode(FUSE_GPIO, OUTPUT);
  digitalWrite(FUSE_GPIO, LOW);
  Fusing();
}

void loop() {
}

void Fusing() {
  digitalWrite(FUSE_GPIO, HIGH);
  delay(400);
  digitalWrite(FUSE_GPIO, LOW);
}