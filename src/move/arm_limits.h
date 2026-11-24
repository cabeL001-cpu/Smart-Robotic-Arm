#pragma once

// 肩部：脉宽 50-1080 µs。11-23 扫点实测：≤320 µs 完全无动作，360-640 流畅，
// 700 起抖动，1000 卡在止挡区抖动；运行窗口仍取 360-640，下限放到 50 只为继续扫端点。
// 肘部：11-23 起把固件窗口放到 50-2700，用于扫描真实机械端点；扫描前 500-2450 均流畅。
constexpr int SHOULDER_MIN_US = 50;
constexpr int SHOULDER_MAX_US = 1080;
constexpr int ELBOW_MIN_US = 50;
constexpr int ELBOW_MAX_US = 2700;