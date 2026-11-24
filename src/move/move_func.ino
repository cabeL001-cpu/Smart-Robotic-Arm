void go() {
  digitalWrite(2, HIGH);
  analogWrite(3, 0);
  digitalWrite(4, HIGH);
  analogWrite(5, 0);
  digitalWrite(8, HIGH);
  analogWrite(6, 0);
  digitalWrite(12, HIGH);
  analogWrite(11, 0);
}

void back() {
  digitalWrite(2, LOW);
  analogWrite(3, 255);
  digitalWrite(4, LOW);
  analogWrite(5, 255);
  digitalWrite(8, LOW);
  analogWrite(6, 255);
  digitalWrite(12, LOW);
  analogWrite(11, 255);
}

void stop() {
  digitalWrite(2, LOW);
  analogWrite(3, 0);
  digitalWrite(4, LOW);
  analogWrite(5, 0);
  digitalWrite(8, LOW);
  analogWrite(6, 0);
  digitalWrite(12, LOW);
  analogWrite(11, 0);
}

void moveLeft() {
  digitalWrite(2, LOW);
  analogWrite(3, 255);
  digitalWrite(4, LOW);
  analogWrite(5, 255);
  digitalWrite(8, HIGH);
  analogWrite(6, 0);
  digitalWrite(12, HIGH);
  analogWrite(11, 0);
}

void moveRight() {
  digitalWrite(2, HIGH);
  analogWrite(3, 0);
  digitalWrite(4, HIGH);
  analogWrite(5, 0);
  digitalWrite(8, LOW);
  analogWrite(6, 255);
  digitalWrite(12, LOW);
  analogWrite(11, 255);
}

void moveForwardLeft() {
  digitalWrite(2, LOW);
  analogWrite(3, 0);
  digitalWrite(4, LOW);
  analogWrite(5, 0);
  digitalWrite(8, HIGH);
  analogWrite(6, 0);
  digitalWrite(12, HIGH);
  analogWrite(11, 0);
}

void moveForwardRight() {
  digitalWrite(2, HIGH);
  analogWrite(3, 0);
  digitalWrite(4, HIGH);
  analogWrite(5, 0);
  digitalWrite(8, LOW);
  analogWrite(6, 0);
  digitalWrite(12, LOW);
  analogWrite(11, 0);
}

void moveBackLeft() {
  digitalWrite(2, LOW);
  analogWrite(3, 255);
  digitalWrite(4, LOW);
  analogWrite(5, 255);
  digitalWrite(8, LOW);
  analogWrite(6, 0);
  digitalWrite(12, LOW);
  analogWrite(11, 0);
}

void moveBackRight() {
  digitalWrite(2, LOW);
  analogWrite(3, 0);
  digitalWrite(4, LOW);
  analogWrite(5, 0);
  digitalWrite(8, LOW);
  analogWrite(6, 255);
  digitalWrite(12, LOW);
  analogWrite(11, 255);
}

void rotateLeft() {
  digitalWrite(2, HIGH);
  analogWrite(3, 0);
  digitalWrite(4, LOW);
  analogWrite(5, 255);
  digitalWrite(8, LOW);
  analogWrite(6, 255);
  digitalWrite(12, HIGH);
  analogWrite(11, 0);
}

void rotateRight() {
  digitalWrite(2, LOW);
  analogWrite(3, 255);
  digitalWrite(4, HIGH);
  analogWrite(5, 0);
  digitalWrite(8, HIGH);
  analogWrite(6, 0);
  digitalWrite(12, LOW);
  analogWrite(11, 255);
}

void turnRightWithLeftWheels() {
  digitalWrite(2, LOW);
  analogWrite(3, 0);
  digitalWrite(4, HIGH);
  analogWrite(5, 0);
  digitalWrite(8, HIGH);
  analogWrite(6, 0);
  digitalWrite(12, LOW);
  analogWrite(11, 0);
}

void turnRightWithFrontWheels() {
  digitalWrite(2, LOW);
  analogWrite(3, 0);
  digitalWrite(4, HIGH);
  analogWrite(5, 0);
  digitalWrite(8, LOW);
  analogWrite(6, 0);
  digitalWrite(12, LOW);
  analogWrite(11, 255);
}

void m1Forward() {
  digitalWrite(2, HIGH);
  analogWrite(3, 0);
}

void m2Forward() {
  digitalWrite(4, HIGH);
  analogWrite(5, 0);
}

void m3Forward() {
  digitalWrite(8, HIGH);
  analogWrite(6, 0);
}

void m4Forward() {
  digitalWrite(12, HIGH);
  analogWrite(11, 0);
}

// 扩展板按 M1 至 M4 的次序配置，每个电机占用一个方向脚和一个 PWM 脚。
void initMotorPins() {
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
}
