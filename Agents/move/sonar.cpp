#include "sonar.h"
#include <Arduino.h>
#include <Wire.h>
#include <string.h>

namespace sonar {
namespace {
// 实车探头标签，顺序与 Direction 的四个掩码位一致。
constexpr uint8_t ADDRESS[] = {0x69, 0x72, 0x6C, 0x6A};
constexpr uint8_t COUNT = sizeof(ADDRESS) / sizeof(ADDRESS[0]);
const char *const NAMES[] = {"FRONT", "BACK", "LEFT", "RIGHT"};
constexpr uint8_t REG = 0x02;
constexpr uint16_t FRESH_MS = 650;  // 正常四路轮询约 432 ms

enum Power { COLD, WARMING, READY };
struct Probe {
  uint16_t mm;       // 0 表示缺失、无回波或无效值
  uint32_t timeMs;   // 预热起点，或最近一次已完成测量的触发时刻
  Power power;
};
Probe probes[COUNT] = {};
enum Phase { TRIGGER, SELECT_REGISTER, READ_ECHO };
Phase phase = TRIGGER;
uint8_t index = 0;
uint32_t nextMs, pingMs;

bool writeRegister(int command = -1) {
  Wire.beginTransmission(ADDRESS[index]);
  Wire.write(REG);
  if (command >= 0) Wire.write(static_cast<uint8_t>(command));
  return Wire.endTransmission() == 0;
}

void advance() {
  index = (index + 1) % COUNT;
  phase = TRIGGER;
  nextMs = millis() + 10;  // 错开发声，避免相邻探头串扰
}

void busFailure() {
  probes[index] = {};  // 撤销旧读数，恢复供电后重新初始化本探头
  advance();
}

uint16_t freshMm(uint8_t i) {
  return millis() - probes[i].timeMs <= FRESH_MS ? probes[i].mm : 0;
}

void printAddress(uint8_t i) {
  Serial.print(F("0x"));
  Serial.print(ADDRESS[i], HEX);
}
}  // namespace

void begin() {
  nextMs = millis();
  Wire.begin();
  Wire.setWireTimeout(10000, true);  // 一次总线事务最多等待约 10 ms
}

void poll() {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - nextMs) < 0) return;
  Probe &p = probes[index];

  if (phase == TRIGGER) {
    if (p.power == COLD) {
      if (writeRegister(0x71)) {
        p.power = WARMING;
        p.timeMs = millis();
      }
      advance();
      return;
    }
    if (p.power == WARMING && now - p.timeMs < 2000) {
      advance();
      return;
    }
    p.power = READY;
    if (!writeRegister(0xBC)) {
      busFailure();
      return;
    }
    pingMs = millis();
    phase = SELECT_REGISTER;
    nextMs = pingMs + 88;
    return;
  }

  if (phase == SELECT_REGISTER) {
    if (!writeRegister()) {
      busFailure();
      return;
    }
    phase = READ_ECHO;
    nextMs = millis() + 10;
    return;
  }

  if (Wire.requestFrom(ADDRESS[index], static_cast<uint8_t>(2)) != 2) {
    busFailure();
    return;
  }
  const uint16_t high = Wire.read();
  const uint16_t mm = (high << 8) | Wire.read();  // 两字节大端，单位毫米
  p.mm = mm < 6000 ? mm : 0;  // 0xBC 档的有效距离小于 6 m
  p.timeMs = pingMs;
  advance();
}

int8_t blockedDirection(uint8_t mask) {
  for (uint8_t i = 0; i < COUNT; ++i) {
    if ((mask & (1 << i)) && freshMm(i) <= STOP_MM) return i;
  }
  return -1;
}

uint16_t distanceMm(uint8_t i) { return freshMm(i); }

void reportBlocked(uint8_t i, uint16_t mm) {
  Serial.print(mm ? F("ERR OBSTACLE ") : F("ERR SONAR "));
  Serial.print(NAMES[i]);
  if (mm) {
    Serial.print(' ');
    Serial.print(mm);
  }
  Serial.println();
}

void handleCommand(const char *line, bool moving) {
  if (strcmp(line, "SONAR") == 0) {
    Serial.print(F("SONAR"));
    for (uint8_t i = 0; i < COUNT; ++i) {
      Serial.print(' ');
      printAddress(i);
      Serial.print('=');
      const uint16_t mm = freshMm(i);
      if (mm) Serial.print(mm);
      else Serial.print(F("NA"));
    }
    Serial.println();
  } else if (strcmp(line, "SONAR SCAN") == 0) {
    if (moving) {
      Serial.println(F("ERR BUSY"));
      return;
    }
    // 只探测四个标签的 ACK，既不触发发声也不修改探头地址。
    Serial.print(F("SCAN"));
    for (uint8_t i = 0; i < COUNT; ++i) {
      Wire.beginTransmission(ADDRESS[i]);
      if (Wire.endTransmission() == 0) {
        Serial.print(' ');
        printAddress(i);
      }
    }
    Serial.println();
  } else {
    Serial.println(F("ERR ACTION"));
  }
}
}  // namespace sonar
