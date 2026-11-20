#include <Servo.h>
#include "arm_limits.h"

// 肩部接扩展板 D13 信号针，肘部接 Uno 数字 D10。D10 同时控制扩展板 VOUT，须保持 VOUT 空载。
const uint8_t SHOULDER_PIN = 13;
const uint8_t ELBOW_PIN = 10;
Servo shoulderServo;
Servo elbowServo;

void setArmPulses(char *arguments) {
  if (SHOULDER_MIN_US < 500 || SHOULDER_MAX_US > 2500 || SHOULDER_MIN_US > SHOULDER_MAX_US ||
      ELBOW_MIN_US < 500 || ELBOW_MAX_US > 2500 || ELBOW_MIN_US > ELBOW_MAX_US) {
    Serial.println(F("ERR ARM CALIBRATION"));
    return;
  }
  char *separator;
  long shoulderUs = strtol(arguments, &separator, 10);
  char *end;
  long elbowUs = strtol(separator, &end, 10);
  if (*separator != ' ' || *end != '\0' ||
      shoulderUs < SHOULDER_MIN_US || shoulderUs > SHOULDER_MAX_US ||
      elbowUs < ELBOW_MIN_US || elbowUs > ELBOW_MAX_US) {
    Serial.println(F("ERR ARM"));
    return;
  }
  // attach 会启用定时中断。先同时装入两路合法脉宽，再允许首个脉冲发出。
  noInterrupts();
  if (!shoulderServo.attached()) {
    shoulderServo.attach(SHOULDER_PIN, 500, 2500);
    elbowServo.attach(ELBOW_PIN, 500, 2500);
  }
  shoulderServo.writeMicroseconds(shoulderUs);
  elbowServo.writeMicroseconds(elbowUs);
  interrupts();
  Serial.println(F("OK ARM"));
}
