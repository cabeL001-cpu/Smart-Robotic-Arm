# 银立方机器人系统集成

当前可运行的链路为 **树莓派 Python → 串口 → Arduino Uno → 扩展板 M1 至 M4**。上层程序在 `Agents` 目录导入 `MHS`，按车体方向和毫秒数控制底盘。每次调用等待 Uno 确认停车。

```python
import MHS

MHS.move("go", 250, port="/dev/ttyACM0")
```

| 模块 | 接口与集成状态 | 模块说明 |
| --- | --- | --- |
| move 底盘 | `MHS.move(action, duration_ms, port=...)`，提供树莓派命令行与 Uno 固件 | [接线、安装、点动和串口协议](move/README.md) |
| arm 机械臂 | 规划接收 0 至 8 的目标格，本阶段仅记录接口边界 | [机械臂说明](arm/README.md) |
| vision 视觉与定位 | 规划提供车体位姿和格心测量，本阶段仅记录所需观测 | [视觉与定位说明](vision/README.md) |
| listen_speak 语音 | 已有独立的识别和播报接口，底盘可单独运行 | [语音接口与使用方法](listen_speak/README.md) |

系统约定上层的 `("A", "arrive")` 表示前往停靠点，`("4", "arrive")` 表示将机械臂末端移到 4 号格。按地点导航需要先取得位姿，再把路径转换为底盘动作。当前的 `MHS.move("go", 250)` 只执行一次定时前进，回执可用于安排下一步动作。现有语音解析器的输出范围见其模块说明。

在本机先进入 `Agents`，运行 `python -m move go --ms 250 --dry-run` 可预览指令。实车使用时，先为 Uno 准备固件，再在树莓派选择实际串口。[四轮底盘操作步骤](move/README.md)
