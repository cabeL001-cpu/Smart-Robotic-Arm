// 以虚拟时钟运行真实草图，检查电机输出、线协议、测距时序与故障恢复。
#include "Arduino.h"
#include "Wire.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include "move_func.ino"
#include "arm_control.ino"
#include "move.ino"

uint32_t clockMs = UINT32_MAX - 3200;
int pins[16] = {};
bool interruptsEnabled = true;
TestSerial Serial;
TestWire Wire;

std::string output() {
  std::string result = Serial.output.str();
  Serial.output.str("");
  return result;
}
std::string command(std::string text) {
  text += "\r\n";
  Serial.input.insert(Serial.input.end(), text.begin(), text.end());
  while (Serial.available()) loop();
  return output();
}
void tick(uint32_t duration) {
  for (uint32_t i = 0; i < duration; ++i) { ++clockMs; loop(); }
}
bool stopped() {
  return std::all_of(pins, pins + 16, [](int value) { return value == 0; });
}

int main() {
  const uint8_t addresses[] = {0x69, 0x72, 0x6C, 0x6A};
  const char *const sides[] = {"FRONT", "BACK", "LEFT", "RIGHT"};
  for (auto a : addresses) Wire.probes[a].online = true;
  setup();
  assert(command("GO 1000") == "ERR SONAR FRONT\n" && stopped());
  assert(command("SONAR SCAN") == "SCAN 0x69 0x72 0x6C 0x6A\n");
  tick(1500);
  assert(command("STOP") == "OK STOP\n");
  assert(command("SONAR") == "SONAR 0x69=NA 0x72=NA 0x6C=NA 0x6A=NA\n");
  tick(1450);
  assert(command("SONAR") == "SONAR 0x69=1000 0x72=1000 0x6C=1000 0x6A=1000\n");
  assert(command("GO 500") == "OK MOVE\n" && !stopped());
  assert(command("SONAR SCAN") == "ERR BUSY\n");
  assert(command("BACK 50") == "ERR BUSY\n");
  tick(499);  // 跨 millis() 回绕，按原时长停车
  assert(!stopped());
  tick(1);
  assert(output() == "DONE\n" && stopped());

  const struct { const char *name; int mask; } motions[] = {
    {"GO",1}, {"BACK",2}, {"LEFT",4}, {"RIGHT",8}, {"FORWARD-LEFT",5}, {"FORWARD-RIGHT",9},
    {"BACK-LEFT",6}, {"BACK-RIGHT",10}, {"ROTATE-LEFT",15}, {"ROTATE-RIGHT",15},
    {"TURN-RIGHT-LEFT-WHEELS",15}, {"TURN-RIGHT-FRONT-WHEELS",15},
    {"M1",15}, {"M2",15}, {"M3",15}, {"M4",15}
  };
  for (int i = 0; i < 4; ++i) {
    Wire.probes[addresses[i]].distance = sonar::STOP_MM;
    tick(500);
    for (const auto &motion : motions) {
      std::string reply = command(motion.name);
      if (motion.mask & (1 << i)) {
        assert(reply == "ERR OBSTACLE " + std::string(sides[i]) + " "
                        + std::to_string(sonar::STOP_MM) + "\n" && stopped());
      } else {
        assert(reply == "OK MOVE\n" && !stopped());
        assert(command("STOP") == "OK STOP\n" && stopped());
      }
    }
    Wire.probes[addresses[i]].distance = 1000;
  }
  tick(500);
  assert(command("GO 5000") == "OK MOVE\n");
  Wire.probes[0x69].distance = 210;
  tick(500);
  assert(output() == "ERR OBSTACLE FRONT 210\n" && stopped());
  Wire.probes[0x69].distance = 1234;
  tick(5200);
  assert(output().empty() && stopped());  // 不自启，也不补发 DONE
  assert(command("GO 4000") == "OK MOVE\n");
  Wire.probes[0x69].online = false;
  tick(500);
  assert(output() == "ERR SONAR FRONT\n" && stopped());
  assert(command("SONAR SCAN") == "SCAN 0x72 0x6C 0x6A\n");
  assert(command("BACK 10") == "OK MOVE\n");
  tick(10);
  assert(output() == "DONE\n");
  Wire.probes[0x69].online = true;
  tick(3100);
  assert(command("SONAR").find("0x69=1234") != std::string::npos);
  Wire.probes[0x69].replyBytes = 1;
  tick(600);
  assert(command("GO 10") == "ERR SONAR FRONT\n");
  Wire.probes[0x69].replyBytes = 2;
  for (uint16_t bad : {0, 6000, 65535}) {
    Wire.probes[0x69].distance = bad;
    tick(3100);
    assert(command("GO 10") == "ERR SONAR FRONT\n");
  }
  Wire.probes[0x69].distance = sonar::STOP_MM + 1;
  tick(500);
  assert(command("GO 5000") == "OK MOVE\n");
  clockMs += 700;  // 即使旧读数显示畅通，采样中断后也应停车
  loop();
  assert(output() == "ERR SONAR FRONT\n" && stopped());

  if (SHOULDER_MIN_US == 0 || ELBOW_MIN_US == 0) {
    assert(command("ARM 1500 1500") == "ERR ARM CALIBRATION\n");
    assert(!shoulderServo.active && !elbowServo.active);
  } else {
    auto joints = [](int a, int b) {
      return command("ARM " + std::to_string(a) + " " + std::to_string(b));
    };
    assert(joints(SHOULDER_MIN_US - 1, ELBOW_MIN_US) == "ERR ARM\n");
    assert(!shoulderServo.active && !elbowServo.active);
    assert(joints(SHOULDER_MIN_US, ELBOW_MAX_US) == "OK ARM\n");
    assert(shoulderServo.signalPin == 13 && elbowServo.signalPin == 10);
    assert(shoulderServo.pulse == SHOULDER_MIN_US && elbowServo.pulse == ELBOW_MAX_US);
    assert(joints(SHOULDER_MAX_US, ELBOW_MAX_US + 1) == "ERR ARM\n");
    assert(shoulderServo.pulse == SHOULDER_MIN_US && elbowServo.pulse == ELBOW_MAX_US);
    assert(interruptsEnabled);
  }
  std::cout << "SR09 protocol, 16 motions, interlocks and servo limits OK\n";
}
