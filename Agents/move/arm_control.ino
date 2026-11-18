#include <Servo.h>

// 扩展板的两个舵机信号口。与 M1 至 M4 使用的八个引脚互不重叠。
const uint8_t SHOULDER_PIN = 13;
const uint8_t ELBOW_PIN = 7;
Servo shoulderServo;
Servo elbowServo;

void setArmPulses(char *arguments) {
  char *separator;
  long shoulderUs = strtol(arguments, &separator, 10);
  char *end;
  long elbowUs = strtol(separator, &end, 10);
  if (*separator != ' ' || *end != '\0' || shoulderUs < 500 || shoulderUs > 2500 ||
      elbowUs < 500 || elbowUs > 2500) {
    Serial.println(F("ERR ARM"));
    return;
  }
  // 收到首个有效动作后才输出 PWM。之后保持脉宽，轮子的 STOP 仅控制底盘。
  if (!shoulderServo.attached()) {
    shoulderServo.attach(SHOULDER_PIN, 500, 2500);
    elbowServo.attach(ELBOW_PIN, 500, 2500);
  }
  shoulderServo.writeMicroseconds(shoulderUs);
  elbowServo.writeMicroseconds(elbowUs);
  Serial.println(F("OK ARM"));
}
