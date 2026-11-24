#pragma once
#include "Arduino.h"
#include <cassert>

struct Servo {
  bool active = false;
  int signalPin = -1;
  int pulse = 0;
  bool attached() const { return active; }
  void attach(int pin, int, int) {
    // 严禁在默认中位有机会发出脉冲时挂接新舵机。
    assert(!interruptsEnabled);
    signalPin = pin;
    active = true;
  }
  void writeMicroseconds(int us) { assert(active); pulse = us; }
  void detach() { active = false; }
};
