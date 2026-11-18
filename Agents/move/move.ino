// Uno 串口入口。各动作直接调用 move_func.ino 中已调好的电机函数。
const unsigned long BAUD_RATE = 9600;
const uint16_t DEFAULT_DURATION_MS = 1000;

unsigned long motionStartedAt;
uint16_t motionDurationMs = 0;  // 0 表示四轮已停车

void setup() {
  initMotorPins();
  stop();
  Serial.begin(BAUD_RATE);
}

bool runMotion(const char *name) {
  static const struct {
    const char *name;
    void (*run)();
  } motions[] = {
    {"GO", go},                       {"BACK", back},
    {"LEFT", moveLeft},               {"RIGHT", moveRight},
    {"FORWARD-LEFT", moveForwardLeft}, {"FORWARD-RIGHT", moveForwardRight},
    {"BACK-LEFT", moveBackLeft},       {"BACK-RIGHT", moveBackRight},
    {"ROTATE-LEFT", rotateLeft},       {"ROTATE-RIGHT", rotateRight},
    {"TURN-RIGHT-LEFT-WHEELS", turnRightWithLeftWheels},
    {"TURN-RIGHT-FRONT-WHEELS", turnRightWithFrontWheels},
    {"M1", m1Forward}, {"M2", m2Forward}, {"M3", m3Forward}, {"M4", m4Forward}
  };
  for (const auto &motion : motions) {
    if (strcmp(name, motion.name) == 0) {
      motion.run();
      return true;
    }
  }
  return false;
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
  if (!runMotion(line)) {
    Serial.println(F("ERR ACTION"));
    return;
  }
  motionStartedAt = millis();
  motionDurationMs = durationMs;
  Serial.println(F("OK MOVE"));
}

void loop() {
  // 用经过时间判断到期，millis() 回绕时仍成立。计时期间也能接收 STOP。
  if (motionDurationMs != 0 && millis() - motionStartedAt >= motionDurationMs) {
    stop();
    motionDurationMs = 0;
    Serial.println(F("DONE"));
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
