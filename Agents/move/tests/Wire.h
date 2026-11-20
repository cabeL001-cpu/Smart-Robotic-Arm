#pragma once
#include "Arduino.h"
#include <cassert>
#include <vector>

struct TestWire {
  struct Probe {
    bool online = false, initialized = false, selected = false;
    uint16_t distance = 1000;
    uint8_t replyBytes = 2;
    uint32_t initAt = 0, pingAt = 0, selectAt = 0;
  } probes[128];
  uint8_t address, byteIndex;
  std::vector<uint8_t> tx;
  bool hasPing = false;
  uint32_t lastPing = 0;

  void begin() {}
  void setWireTimeout(uint32_t us, bool reset) { assert(us == 10000 && reset); }
  void beginTransmission(uint8_t a) { address = a; tx.clear(); }
  void write(uint8_t v) { tx.push_back(v); }
  int endTransmission() {
    Probe &p = probes[address];
    if (!p.online) return 2;
    if (tx.empty()) return 0;  // 地址探测
    assert(tx[0] == 0x02);
    if (tx.size() == 2 && tx[1] == 0x71) {
      p.initAt = millis();
      p.initialized = true;
    } else if (tx.size() == 2 && tx[1] == 0xBC) {
      assert(p.initialized && millis() - p.initAt >= 2000);
      assert(!hasPing || millis() - lastPing >= 98);
      hasPing = true;
      lastPing = p.pingAt = millis();
      p.selected = false;
    } else {
      assert(tx.size() == 1 && millis() - p.pingAt >= 88);
      p.selected = true;
      p.selectAt = millis();
    }
    return 0;
  }
  uint8_t requestFrom(uint8_t a, uint8_t n) {
    Probe &p = probes[a];
    address = a;
    byteIndex = 0;
    assert(n == 2 && p.selected && millis() - p.selectAt >= 10);
    return p.online ? p.replyBytes : 0;
  }
  int read() {
    uint16_t mm = probes[address].distance;
    return byteIndex++ == 0 ? mm >> 8 : mm & 0xff;
  }
};
extern TestWire Wire;
