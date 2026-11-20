#pragma once
#include "Arduino.h"
#include <cassert>

struct Servo {
  bool active = false;
  int pulse = 0;
  bool attached() const { return active; }
  void attach(int, int, int) {
    // 严禁在默认中位有机会发出脉冲时挂接新舵机。
    assert(!interruptsEnabled);
    active = true;
  }
  void writeMicroseconds(int us) { assert(active); pulse = us; }
};
