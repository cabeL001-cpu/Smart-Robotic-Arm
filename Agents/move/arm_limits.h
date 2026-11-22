#pragma once

// 肩部：脉宽 400-1080 µs，按实测 θ=(p+120)/12 约为 43.3-100°。
// 400 µs 为实测可用下限（100 µs 几乎无响应）；肘部端点待标定，暂用 500-2500。
constexpr int SHOULDER_MIN_US = 400;
constexpr int SHOULDER_MAX_US = 1080;
constexpr int ELBOW_MIN_US = 500;
constexpr int ELBOW_MAX_US = 2500;