#pragma once
#include <stdint.h>

namespace sonar {
// 按车体坐标命名，斜移同时检查两个方向，转动时检查四面。
enum Direction : uint8_t { FRONT = 1, BACK = 2, LEFT = 4, RIGHT = 8, ALL = 15 };
constexpr uint16_t STOP_MM = 300;  // 初始值，按实测车速、轮询周期和制动距离调整

void begin();
void poll();
int8_t blockedDirection(uint8_t mask);   // -1 表示所需方向均可通行
uint16_t distanceMm(uint8_t index);
void reportBlocked(uint8_t index, uint16_t mm);
inline void reportBlocked(uint8_t index) { reportBlocked(index, distanceMm(index)); }
void handleCommand(const char *line, bool moving);
}
