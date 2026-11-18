# move 四轮底盘

树莓派把一个动作和持续时间发给 Uno。Uno 调用原草图中已调好的动作函数，到时将四路电机输出归零并回复树莓派。方向以车体为准，时长的单位是毫秒。

## 现在可以在本机验证

从仓库的 `Agents` 目录执行以下命令。Windows 使用 `python`，树莓派使用 `python3`。预览只使用 Python 标准库。

```bash
python -m move go --ms 250 --dry-run
python -m move stop --dry-run
```

两条命令分别打印 `GO 250` 和 `STOP`。预览用来核对将要发送的指令。若本机已安装 Arduino CLI 和 `arduino:avr` 平台，还可以离线编译 Uno 固件。

```bash
arduino-cli lib install Servo   # 共用 Uno 的机械臂固件依赖，安装一次即可
arduino-cli compile --fqbn arduino:avr:uno move
```

本版本已通过 Uno 目标编译。离线联调核对了 16 个运动动作的四路输出、到时停车、提前停车和断线后到时停车。电机实物转向与车体行进方向留待架空点动和落地试车确认。

## 拿到硬件后运行

1. 将四个车轮架空，沿用调好参数时的 M1 至 M4 接线，接好底盘驱动电源。各电机座和引脚标注见[扩展板图](../image-8.png)。
2. 在 Arduino IDE 打开 `move/move.ino`，安装 Servo 库，同目录保留 `move_func.ino` 和 `arm_control.ino`。选择 **Arduino Uno** 和对应端口，将草图上传到 Uno，然后关闭串口监视器。
3. 用 USB 数据线连接树莓派与 Uno 的板载串口。若沿用 USB 转 TTL 串口模块，使用电平与 Uno 兼容的模块，将模块 TX 接 Uno RX、模块 RX 接 Uno TX，并共地。板载 USB 串口和外接模块任选一条控制路径。
4. 在树莓派上进入本项目的 `Agents` 目录。下列示例创建一个底盘专用 Python 环境，安装 pySerial，并列出串口。若已激活项目原有的 `~/mx` 环境，可直接运行最后两行。

```bash
cd /path/to/Smart-Robotic-Arm/Agents
python3 -m venv ~/move-venv
source ~/move-venv/bin/activate
python3 -m pip install -r move/requirements.txt
python3 -m serial.tools.list_ports -v
```

5. 将下列命令中的 `/dev/ttyACM0` 换成上一步识别到的串口，例如 `/dev/ttyUSB0`。先确认停车通信，再短时点动一个电机，最后让四轮一起前进。

```bash
python3 -m move stop --port /dev/ttyACM0
python3 -m move m1 --ms 200 --port /dev/ttyACM0
python3 -m move go --ms 250 --port /dev/ttyACM0
```

运行 `go` 时，四路电机应同时按标定的前进方向转动约 250 毫秒，随后全部停止。终端收到停车回执后打印「Uno 已确认停车」。依次将 `m1` 换成 `m2`、`m3`、`m4` 可核对每个电机座对应的实物车轮。将车轮落地后再确认车体实际行进方向。

## 可用动作

| 命令 | 动作 |
| --- | --- |
| `go`、`back` | 四轮前进、后退 |
| `left`、`right` | 向左、向右平移 |
| `forward-left`、`forward-right` | 向左前、右前斜移 |
| `back-left`、`back-right` | 向左后、右后斜移 |
| `rotate-left`、`rotate-right` | 原地左旋、右旋 |
| `turn-right-left-wheels`、`turn-right-front-wheels` | 用左侧轮、前轮执行参考草图中的两种右转 |
| `m1`、`m2`、`m3`、`m4` | 单独点动对应电机向前，其余三路保持停车 |
| `stop` | 立即将四路输出置为停车状态 |

`--ms` 接受 1 到 65535 的整数，默认 1000。省略 `--port` 时使用 `/dev/ttyACM0`。运动期间可在当前终端按 **Ctrl+C** 请求提前停车。控制程序独占串口，同一时间由一个进程发指令。

## Python 调用

在 `Agents` 目录运行，或将该目录加入项目的模块搜索路径。一个调用收到 Uno 的停车回执之后，下一个动作才开始。

```python
import MHS

port = "/dev/ttyACM0"
MHS.move("go", 250, port=port)
MHS.move("rotate-left", 150, port=port)
MHS.move("stop", port=port)
```

时间参数控制通电时长。该接口的回执表示 Uno 已执行停车输出，实际位移与转角取决于底盘和地面。

## 引脚与串口约定

| 扩展板电机座 | 方向引脚 | PWM 引脚 |
| --- | --- | --- |
| M1 | D2 | D3 |
| M2 | D4 | D5 |
| M3 | D8 | D6 |
| M4 | D12 | D11 |

每路标定的正转输出为 `HIGH / 0`，反转为 `LOW / 255`，停车为 `LOW / 0`，顺序为方向脚、PWM 脚。各动作直接复用[原电机函数](move_func.ino)。

串口使用 **9600 波特、8N1**。请求是大写 ASCII 文本，以 LF 结束，也接受 CRLF。串口示例中的单词 `GO` 可直接使用，默认运动 1000 毫秒。[串口示例图](../image-9.png)

| 树莓派发送的一行 | Uno 的回执与动作 |
| --- | --- |
| `STOP` | 四轮立即停车，回复 `OK STOP`。该指令也用于连接握手 |
| `GO 250` | 启动前进后回复 `OK MOVE`，250 毫秒到期时停车并回复 `DONE` |
| 运动中收到 `STOP` | 取消当前计时，立即停车并回复 `OK STOP` |
| 运动中收到另一条运动指令 | 回复 `ERR BUSY`，继续原动作直至到期或收到 `STOP` |
| 无效动作、无效时长或超长行 | 分别回复 `ERR ACTION`、`ERR DURATION` 或 `ERR LINE` |

打开串口可能使 Uno 复位，树莓派会在握手期间重发 `STOP`，收到确认后只发送一次运动指令。Uno 使用本机时钟计时，通信中断后仍会在该次动作到期时停车。

若提示「Uno 未确认停车」，核对设备名、两端波特率、固件和收发线。若已收到停车回执但某个轮子没有转动，用单轮命令检查对应电机座、底盘供电和实际接线。

同一 Uno 同时提供关节控制。组合动作通过 `MHS.open_uno` 复用串口，打开时握手停车一次。[机械臂调用示例](../arm/README.md)

底盘位姿可由独立的 IMU 串口与相机提供。[IMU 定位和离线回放](imu/README.md)
