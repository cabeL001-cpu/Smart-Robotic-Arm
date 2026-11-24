#pragma once
// 桌面回归测试使用的最小 Uno 接口。
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <deque>
#include <sstream>
#include <string>

#define F(x) x
constexpr int HEX = 16, LOW = 0, HIGH = 1, OUTPUT = 1;
extern uint32_t clockMs;
extern int pins[16];
extern bool interruptsEnabled;
inline uint32_t millis() { return clockMs; }
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int value) { pins[pin] = value; }
inline void analogWrite(int pin, int value) { pins[pin] = value; }
inline void noInterrupts() { interruptsEnabled = false; }
inline void interrupts() { interruptsEnabled = true; }

struct TestSerial {
  std::ostringstream output;
  std::deque<char> input;
  void begin(unsigned long) {}
  int available() { return static_cast<int>(input.size()); }
  int read() {
    char first = input.front();
    input.pop_front();
    return first;
  }
  template<class T> void print(const T &v) { output << v; }
  void print(uint8_t v, int) { output << std::hex << std::uppercase << unsigned(v) << std::dec; }
  void println() { output << '\n'; }
  void println(const char *s) { print(s); println(); }
};
extern TestSerial Serial;
