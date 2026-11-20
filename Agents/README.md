# 银立方机器人系统集成

实车通过蓝牙把命令交给独立供电的 Uno，蓝牙传输由另一位同学负责。Uno 在本机轮询四个 SR09，驱动四轮及两台 MG995。在 `Agents` 目录导入 `MHS`。底盘正常回执表示已停车，舵机回执表示已设置关节输出。

```python
import os
import MHS

MHS.move("go", 250, port=os.environ["UNO_PORT"])
```

| 模块 | 接口与集成状态 | 模块说明 |
| --- | --- | --- |
| move 底盘 | `MHS.move(action, duration_ms, port=...)`，提供树莓派命令行与 Uno 固件 | [接线、安装、点动和串口协议](move/README.md) |
| 四向避障 | 前 `0x69`、后 `0x72`、左 `0x6C`、右 `0x6A`。`python -m move.sonar read --watch --port "$UNO_PORT"` 查看距离，底盘自行拒绝或停止受阻动作 | [探头接线、规则与回执](move/README.md) |
| arm 机械臂 | `MHS.arm(cell, config=cfg)` 在两轴各自最大位置回退 90° 的限位内规划。实测标定导出给 Uno 后执行 | [机械臂说明](arm/README.md) |
| vision 视觉与定位 | 随车单相机同时测量格心误差和场地位姿。`python3 -m vision.robot --cell 4` 可共用同一帧 | [视觉与定位说明](vision/README.md) |
| IMU 定位 | 独立读取 YbImu 串口、静止校准、二维推算与带时间戳的视觉校正 | [实时采样、回放和融合](move/imu/README.md) |
| listen_speak 语音 | 已有独立的识别和播报接口，底盘可单独运行 | [语音接口与使用方法](listen_speak/README.md) |

系统约定上层的 `("A", "arrive")` 表示前往停靠点，`("4", "arrive")` 表示将机械臂末端移到 4 号格。视觉提供米和弧度格式的车体位姿，路径规划可将停靠目标转换为底盘动作。当前的 `MHS.move("go", 250)` 只执行一次定时前进，回执可用于安排下一步动作。现有语音解析器的输出范围见其模块说明。

在本机先进入 `Agents`，运行 `python -m move go --ms 250 --dry-run` 可预览指令。实车使用时，先为 Uno 准备固件，再在树莓派选择实际串口。[四轮底盘操作步骤](move/README.md)
