// Uno 串口入口。各动作直接调用 move_func.ino 中已调好的电机函数。
#include "sonar.h"

const unsigned long BAUD_RATE = 9600;
const uint16_t DEFAULT_DURATION_MS = 1000;

uint32_t motionStartedAt;
uint16_t motionDurationMs = 0;  // 0 表示四轮已停车
uint8_t motionDirections = 0;

void setup() {
  initMotorPins();
  stop();
  Serial.begin(BAUD_RATE);
  sonar::begin();
}

void startMotion(const char *name, uint16_t durationMs) {
  using namespace sonar;
  static const struct {
    const char *name;
    void (*run)();
    uint8_t directions;
  } motions[] = {
    {"GO", go, FRONT}, {"BACK", back, BACK},
    {"LEFT", moveLeft, LEFT}, {"RIGHT", moveRight, RIGHT},
    {"FORWARD-LEFT", moveForwardLeft, FRONT | LEFT},
    {"FORWARD-RIGHT", moveForwardRight, FRONT | RIGHT},
    {"BACK-LEFT", moveBackLeft, BACK | LEFT},
    {"BACK-RIGHT", moveBackRight, BACK | RIGHT},
    {"ROTATE-LEFT", rotateLeft, ALL}, {"ROTATE-RIGHT", rotateRight, ALL},
    {"TURN-RIGHT-LEFT-WHEELS", turnRightWithLeftWheels, ALL},
    {"TURN-RIGHT-FRONT-WHEELS", turnRightWithFrontWheels, ALL},
    {"M1", m1Forward, ALL}, {"M2", m2Forward, ALL},
    {"M3", m3Forward, ALL}, {"M4", m4Forward, ALL}
  };
  for (const auto &motion : motions) {
    if (strcmp(name, motion.name) == 0) {
      int8_t blocker = blockedDirection(motion.directions);
      if (blocker >= 0) {
        reportBlocked(blocker);
        return;
      }
      motionDirections = motion.directions;
      motionStartedAt = millis();
      motionDurationMs = durationMs;
      motion.run();
      Serial.println(F("OK MOVE"));
      return;
    }
  }
  Serial.println(F("ERR ACTION"));
}

// 线上时长是十进制毫秒数，范围取自 uint16_t。用 0 表示解析失败。
uint16_t parseDuration(const char *digits) {
  unsigned long value = 0;
  for (; *digits; ++digits) {
    if (*digits < '0' || *digits > '9') return 0;
    value = value * 10 + (*digits - '0');
    if (value > UINT16_MAX) return 0;
  }
  return value;
}

void handleCommand(char *line) {
  if (strcmp(line, "SONAR") == 0 || strncmp(line, "SONAR ", 6) == 0) {
    sonar::handleCommand(line, motionDurationMs != 0);
    return;
  }
  if (strncmp(line, "ARM ", 4) == 0) {
    setArmPulses(line + 4);
    return;
  }
  if (strcmp(line, "STOP") == 0) {
    stop();
    motionDurationMs = 0;
    Serial.println(F("OK STOP"));
    return;
  }

  uint16_t durationMs = DEFAULT_DURATION_MS;
  char *space = strchr(line, ' ');
  if (space != nullptr) {
    *space = '\0';
    durationMs = parseDuration(space + 1);
    if (durationMs == 0) {
      Serial.println(F("ERR DURATION"));
      return;
    }
  }
  if (motionDurationMs != 0) {
    Serial.println(F("ERR BUSY"));
    return;
  }
  startMotion(line, durationMs);
}

void loop() {
  sonar::poll();
  if (motionDurationMs != 0) {
    int8_t blocker = sonar::blockedDirection(motionDirections);
    if (blocker >= 0 || millis() - motionStartedAt >= motionDurationMs) {
      stop();
      motionDurationMs = 0;
      if (blocker >= 0) sonar::reportBlocked(blocker);
      else Serial.println(F("DONE"));
    }
  }

  static char line[40];
  static size_t length = 0;
  if (!Serial.available()) return;
  char ch = Serial.read();
  if (ch == '\n') {
    if (length < sizeof(line)) {
      if (length > 0 && line[length - 1] == '\r') --length;
      line[length] = '\0';
      handleCommand(line);
    } else {
      Serial.println(F("ERR LINE"));
    }
    length = 0;
  } else if (length < sizeof(line)) {
    // 超长行或内含 NUL 的行整行丢弃，下一次换行后重新接收。
    if (ch == '\0') length = sizeof(line);
    else line[length++] = ch;
  }
}
