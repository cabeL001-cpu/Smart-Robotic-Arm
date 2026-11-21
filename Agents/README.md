# 银立方机器人系统集成

全部 Python 命令从 `Agents/` 运行。首次装机先完成接线、依赖安装和 Uno 固件烧录。日常运行由车载电源供电，树莓派经 HC-05 的 `/dev/rfcomm0` 向 Uno 发送命令，YbImu 使用独立 USB 串口。[部署指南](DEPLOY_RASPBERRY_PI.md) · [HC-05 配对与绑定](BLUETOOTH_WORKFLOW.md)

## 验证车载链路

在已配对的树莓派上激活项目虚拟环境。先将四轮架空，关闭轮臂负载电源，保留主控和探头供电。下列命令只停车和读取传感器。

```bash
sh bt_link.sh bind
python -m move stop
python -m move.sonar scan
python -m move.sonar read
```

停车握手成功会显示「Uno 已确认停车。」。扫描应依次列出前 `0x69`、后 `0x72`、左 `0x6C`、右 `0x6A`。待探头预热并取得回波后，最后一条会给出四个方向的毫米读数；缺测显示 `null`。实车点动步骤见底盘说明。

## 模块入口

| 模块 | 入口与输出 | 说明 |
| --- | --- | --- |
| 四轮与避障 | `MHS.move(action, duration_ms)` 请求定时动作，Uno 按方向检查 SR09，到时停车或报告受阻。`python -m move.go_one_second` 会实际请求前进一秒 | [接线、点动与一秒测试](move/README.md) |
| 机械臂 | `MHS.arm(cell, config=cfg)` 规划肩、肘两轴。肩接 D13，肘接 D10，先用实测参数导出 Uno 限位 | [行程标定与验收](arm/README.md) |
| 视觉 | `vision.scan top` 检查棋盘；`vision.arm` 测末端误差；`vision.robot` 输出车体位姿；`vision.align` 通过 Uno 驱动关节。配有前摄时，`vision.scan front` 输出墙标 1–4 的像素坐标 | [相机设置与示例](vision/README.md) |
| IMU | `python -m move.imu` 输出平面位置与航向；`--vision-stdin` 可接收同机 `vision.robot` 的位姿修正 | [采样、记录与回放](move/imu/README.md) |
| 语音 | `listen_speak.listen()` 返回格号和动作，`speak()`、`say_event()` 负责播报 | [语音接口与自测](listen_speak/README.md) |

## 共用 Uno 连接

先配置好蓝牙与探头。在一次 `MHS.open_uno()` 中顺序调用多个接口可以复用停车握手。下面的示例查询前方距离。

```python
import MHS
from move.sonar import read

with MHS.open_uno() as link:
    MHS.move("stop", link=link)
    print(read(link)["front"]["mm"])
```

`OK ARM` 表示 Uno 已设置两路舵机输出，末端是否到达格心由视觉测量。语音目前可返回 `("4", "arrive")` 一类的格号指令，调用方负责选择动作并传入标定。A–D 停靠属于上层导航目标，现有底盘接口接受方向和时长。
