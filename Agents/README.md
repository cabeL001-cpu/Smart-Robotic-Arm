# 银立方机器人系统集成

树莓派通过一块 Uno 驱动四轮和两台 MG995。在 `Agents` 目录导入 `MHS`。组合动作使用 `MHS.open_uno` 复用串口，底盘回执表示已停车，舵机回执表示关节指令已接受。

```python
import MHS

MHS.move("go", 250, port="/dev/ttyACM0")
```

| 模块 | 接口与集成状态 | 模块说明 |
| --- | --- | --- |
| move 底盘 | `MHS.move(action, duration_ms, port=...)`，提供树莓派命令行与 Uno 固件 | [接线、安装、点动和串口协议](move/README.md) |
| arm 机械臂 | `MHS.arm(cell)` 在对齐工位规划两关节并通过共用 Uno 执行 | [机械臂说明](arm/README.md) |
| vision 视觉与定位 | 随车单相机同时测量格心误差和场地位姿。`python3 -m vision.robot --cell 4` 可共用同一帧 | [视觉与定位说明](vision/README.md) |
| listen_speak 语音 | 已有独立的识别和播报接口，底盘可单独运行 | [语音接口与使用方法](listen_speak/README.md) |

系统约定上层的 `("A", "arrive")` 表示前往停靠点，`("4", "arrive")` 表示将机械臂末端移到 4 号格。视觉提供米和弧度格式的车体位姿，路径规划可将停靠目标转换为底盘动作。当前的 `MHS.move("go", 250)` 只执行一次定时前进，回执可用于安排下一步动作。现有语音解析器的输出范围见其模块说明。

在本机先进入 `Agents`，运行 `python -m move go --ms 250 --dry-run` 可预览指令。实车使用时，先为 Uno 准备固件，再在树莓派选择实际串口。[四轮底盘操作步骤](move/README.md)
