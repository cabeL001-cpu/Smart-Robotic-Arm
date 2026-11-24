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

停车握手成功会显示「Uno 已确认停车。」。扫描应依次列出前 `0x6C`、后 `0x72`、左 `0x6F`、右 `0x6A`。待探头预热并取得回波后，最后一条会给出四个方向的毫米读数；缺测显示 `null`。实车点动步骤见底盘说明。

## 模块入口

| 模块 | 入口与输出 | 说明 |
| --- | --- | --- |
| 坐标导航 | `MHS.open_navigation` 在一次启动零点下依次执行 `nav.go_to_dock("A")` 或 `nav.go_to(x_m,y_m)`。IMU 粗定位、前摄整圈扫描和视觉微调共用 Uno 四向锁停 | [小车标定与运行](move/CALIBRATION.md) · [接口与参数](API.md) |
| 四轮与避障 | `MHS.move(action, duration_ms)` 请求定时动作，Uno 按方向检查 SR09，到时停车或报告受阻。`python -m move.go_one_second` 会实际请求前进一秒；`python -m move.south_route` 跑南侧验收路线 | [接线、点动与一秒测试](move/README.md) |
| 机械臂 | `MHS.arm(cell, config=cfg)` 规划肩、肘两轴。肩接 D13，肘接 D10，先用实测参数导出 Uno 限位 | [行程标定与验收](arm/README.md) |
| 视觉 | 顶摄 `vision.arm`、`vision.robot`、`vision.align` 分别测末端、测车心和闭环到格；`vision.servo` 以印刷数字质心为格目标做臂雅可比迭代（`--closed` 为不依赖预置位的全闭环，适合九宫格相对车体位移的情况），末端可解码时用标签四角、否则用黑框四边形中心，`vision.drive` 固定臂走车微调够不到的格，`vision.reposition` 用四个印刷角标把车微调回 11-23 标定 0/3/6/7/8 时的位姿，`vision.place` 按可达性自动分流两种闭环。前摄 `vision.scan front` 读墙标像素，`vision.front` 由地图和内外参输出米制车心 | [相机设置与示例](vision/README.md) · [机械臂标定](arm/CALIBRATION.md) |
| IMU | `python -m move.imu` 输出平面位置与航向；`--vision-stdin` 可接收同机 `vision.robot` 的位姿修正；`python -m move.heading` 用累计航向分段修正直行偏航 | [采样、记录与回放](move/imu/README.md) |
| 语音 | `MHS.listen()` 返回格号和动作，`MHS.say(text)` 播报文本，`listen_speak.say_event()` 播报预设事件 | [语音接口与自测](listen_speak/README.md) |

## 共用 Uno 连接

先配置好蓝牙与探头。在一次 `MHS.open_uno()` 中顺序调用多个接口可以复用停车握手。下面的示例查询前方距离。

```python
import MHS
from move.sonar import read

with MHS.open_uno() as link:
    MHS.move("stop", link=link)
    print(read(link)["front"]["mm"])
```

`OK ARM` 表示 Uno 已设置两路舵机输出，末端是否到达格心由顶摄测量。语音当前返回 `("4","arrive")` 等格号，`MHS.dispatch` 可将它交给已标定工位的机械臂。调用方也可直接传 `("A","arrive")` 和已打开的 `navigator`，启动字母点导航。若需在导航会话中操作手臂，使用 `nav.shared_link()` 或 `MHS.dispatch` 顺序复用连接。[调用和返回示例](API.md)
