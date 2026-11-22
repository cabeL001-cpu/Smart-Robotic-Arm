# move 四轮底盘

树莓派把一个动作和持续时间发给 Uno。Uno 调用原草图中已调好的动作函数，到时将四路电机输出归零并回复树莓派。方向以车体为准，时长的单位是毫秒。

## 启动零点、停靠点与任意坐标

标定 IMU、前摄、墙面标签和轮速后，从 `Agents` 运行 `python -m move.navigation --config "$CFG/navigation/nav.json" --dock A B C D`。`--xy X Y --frame start` 使用启动时车心为零点的米坐标，`--frame field` 使用地图坐标。程序先惯性粗走，再按 IMU 实测转满一圈，停车取新帧并微调。[小车标定与锁停恢复](CALIBRATION.md) · [Python 与 Uno 指令](../API.md)

## 四向 SR09 避障

IIC 扩展板任选一个 4P 口接 Uno 主控，另四口各接一个 SR09。主控侧 `SDA→A4`、`SCL→A5`、`VCC→V`、`GND→G`。连接前核对端子凸起和两端的 GND 丝印，课件这组 4P 线的白线接探头 GND。四个插口电气并联，方向由实物标签确定。地址是七位值，保持探头原地址。

| 以车头为前 | 前 FRONT | 后 BACK | 左 LEFT | 右 RIGHT |
| --- | --- | --- | --- | --- |
| SR09 地址 | `0x6C` | `0x72` | `0x6F` | `0x6A` |

Uno 上电向各探头 `0x02` 寄存器写 `0x71` 并分别等待 2 秒预热。随后错开四路发声，写 `0xBC` 启动一次测距，88 ms 后选寄存器 `0x02`，再等 100 ms 读两个字节，高字节在前，单位毫米。等待由 `millis()` 调度，控制循环可继续接收 `STOP`。固件给每次 IIC 事务设 **50 ms** 上限：实测 SR09 选寄存器时会一直拉住 SCL，直到超时复位总线才放行（原 10 ms 超时会把这次正常等待判成故障并反复复位探头，导致四路全部 `NA`）；而完全不设超时又会让卡住的探头永久阻塞控制循环。选寄存器这次写超时不判故障，以随后的读出长度为准。四路一轮约 1 秒。协议依据课程 PPT 第 33、38–51 页及配套 `I2C_SR09_Demo.ino`。[固件测距实现](sonar.cpp)

| 动作 | 须有新鲜净空读数的方向 |
| --- | --- |
| 前进、后退、左移、右移 | 对应的单个方向 |
| 斜移 | 两个行进分量，例如左前移检查前和左 |
| 旋转、两种偏转、单轮点动 | 四面 |
| `STOP` | 随时执行 |

手动模式在启动前与运动中按上表检查所需方向。距离 **≤300 mm** 时拦截或停车，回复如 `ERR OBSTACLE FRONT 240`。未预热、NACK、短读、0、超量程（≥6000 mm）或读数已超过 **1500 ms**，回复如 `ERR SONAR FRONT`。手动待命时可对畅通方向发新动作。导航模式在行驶、扫描及停车等待时均检查四面，记录首次受阻原因并锁住所有车轮动作。移走障碍后由操作员确认四向净空并显式 `NAV RESET`。Python 调用方可捕获 `move.MotionBlocked` 获取原因。

**阈值需按实车验证。** 一轮四探头测量约 0.43 秒，300 mm 是初始设置。按实测最高车速乘以最坏观测延迟，加上车辆制动距离、探头相对车沿的后缩距离和余量，确定 `STOP_MM` 并重新编译。验收时将轮子架空，在每只探头前放置可产生回波的平板，先在阈值外，再在阈值内观察。架空点动或位移验收时，可执行 `python -m move --avoid off` 临时关闭判停；Uno 重新上电后恢复为开启。开阔场地无有效回波时该方向也会保持停车。[阈值定义](sonar.h)

### 单独读数

在树莓派的 `Agents` 中运行。先配对 HC-05 并执行 `sh bt_link.sh bind`。下列命令优先读取环境变量 `UNO_PORT`，默认端点为 `/dev/rfcomm0`。每次独立打开端点会进行停车握手。[蓝牙配置](../BLUETOOTH_WORKFLOW.md)

~~~bash
python -m move.sonar scan
python -m move.sonar read --watch
~~~

正常扫描显示 `0x6C 0x72 0x6F 0x6A`。扫描只做地址探测；恰好撞上某探头正在测距的 88 ms 窗口时，该地址会偶发漏报，多扫一两次即可，以 `SONAR` 的四向读数为准。第二条每 0.25 秒显示缓存快照。`front/back/left/right` 各有十六进制 `address` 及 `mm`，缺测为 `null`。在 `MHS.open_uno()` 返回的连接上可调用 `move.sonar.scan(link)`、`move.sonar.read(link)`。手动查询时发送 `SONAR SCAN` 或 `SONAR` 加换行。

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

在有 C++ 编译器的开发机上，以下命令用虚拟时钟和四个模拟探头运行真实底盘固件，覆盖 16 个动作的启动和中途避障、读数过期、掉线恢复以及计时回绕。Python 测试验证上层收到同样的停车原因。

~~~bash
c++ -std=c++17 -I move/tests -I move move/tests/sonar_test.cpp move/sonar.cpp -o /tmp/sr09-test
/tmp/sr09-test
python -m unittest move.test_sonar
~~~

## 拿到硬件后运行

1. 将四个车轮架空，沿用调好参数时的 M1 至 M4 接线，接好底盘驱动电源。各电机座和引脚标注见[扩展板图](../image-8.png)。
2. 使用支持 `Wire.setWireTimeout` 的 Arduino AVR Boards（已测试 1.8.8）和 Servo 库，打开 `move/move.ino` 并编译完整草图，同目录保留 `move_func.ino` 和 `arm_control.ino`。USB 用于烧录，上传成功后关闭串口监视器并断开 USB。
3. 把已配好的 HC-05 插到扩展板标注 `5V GND RX TX` 的蓝牙座，该座接 Uno 硬件串口 D0/D1，固件无需改动，字节协议仍是原来的 ASCII 行。树莓派侧先按[部署指南第 3.1 节](../DEPLOY_RASPBERRY_PI.md)用 `bluetoothctl` 完成配对，再运行 `sh bt_link.sh bind` 得到 `/dev/rfcomm0`。板载 USB 口与蓝牙座共用 D0/D1，烧录时必须拔下模块，运行时不要插 USB 数据线。Uno 和轮臂负载使用车载电源。
4. 在树莓派的 `Agents` 目录安装 pySerial。已有项目虚拟环境时直接在其中运行安装命令，否则按下列示例创建专用环境。`serial.tools.list_ports` 只列 USB 串口，不会显示蓝牙串口，用 `ls -l /dev/rfcomm0` 确认绑定。

```bash
cd /path/to/Smart-Robotic-Arm/Agents
python3 -m venv ~/move-venv
source ~/move-venv/bin/activate
python3 -m pip install -r move/requirements.txt
ls -l /dev/rfcomm0
```

5. 下列命令默认使用 `UNO_PORT` 环境变量，未设置时用 `/dev/rfcomm0`。先确认停车通信，按上节检查四向测距，再短时点动一个电机，最后让四轮一起前进。

```bash
python3 -m move stop
python3 -m move m1 --ms 200
python3 -m move go --ms 250
```

运行 `go` 时，四路电机应同时按标定的前进方向转动约 250 毫秒，随后全部停止。终端收到停车回执后打印「Uno 已确认停车」。依次将 `m1` 换成 `m2`、`m3`、`m4` 可核对每个电机座对应的实物车轮。将车轮落地后再确认车体实际行进方向。

**前进一秒。** 确认停车握手、前向测距和短时点动均正常后，从 `Agents` 运行 `python -m move.go_one_second`。脚本实际请求 `GO 1000`，使用同一套避障和到时停车逻辑。若只想预览指令，运行 `python -m move go --ms 1000 --dry-run`。

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

`--ms` 接受 1 到 65535 的整数，默认 1000。省略 `--port` 时先读 `UNO_PORT` 环境变量，没有则用 `/dev/rfcomm0`。运动期间可在当前终端按 **Ctrl+C** 请求提前停车。控制程序独占串口，同一时间由一个进程发指令。

## Python 调用

在 `Agents` 目录运行，或将该目录加入项目的模块搜索路径。一个调用收到 Uno 的停车回执之后，下一个动作才开始。

```python
import os
import MHS

port = os.environ.get("UNO_PORT", "/dev/rfcomm0")
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

串口使用 **9600 波特、8N1**，这是 HC-05 的 UART 波特率，蓝牙链路两端一致。请求是大写 ASCII 文本，以 LF 结束，也接受 CRLF。串口示例中的单词 `GO` 可直接使用，默认运动 1000 毫秒。[串口示例图](../image-9.png)

| 树莓派发送的一行 | Uno 的回执与动作 |
| --- | --- |
| `STOP` | 四轮立即停车，回复 `OK STOP`。该指令也用于连接握手 |
| `GO 250` | 启动前进后回复 `OK MOVE`，250 毫秒到期时停车并回复 `DONE` |
| `SONAR` / `SONAR SCAN` | 返回四向快照 / 预设地址中应答的地址。扫描仅在停车时执行 |
| 所需方向被挡或缺测 | 起动前拒绝动作，行进中先停车再回复 `ERR OBSTACLE 方向 毫米` 或 `ERR SONAR 方向`，本次结束 |
| 运动中收到 `STOP` | 取消当前计时，立即停车并回复 `OK STOP` |
| 运动中收到另一条运动指令 | 回复 `ERR BUSY`，继续原动作直至到期或收到 `STOP` |
| 无效动作、无效时长或超长行 | 分别回复 `ERR ACTION`、`ERR DURATION` 或 `ERR LINE` |

树莓派在建立控制连接时用 `STOP` 握手。蓝牙链路刚建立时开头几个字节可能丢失，树莓派会在握手期间重发 `STOP`，收到确认后只发送一次运动指令；走 USB 台架调试时打开串口还可能使 Uno 复位。Uno 使用本机时钟计时并本地负责测距，通信中断后仍会在该次动作到期时停车，且当前动作始终受避障约束。

若提示「Uno 未确认停车」，核对 `/dev/rfcomm0` 是否已绑定、两端波特率、固件和模块是否插在蓝牙座上。若已收到停车回执但某个轮子没有转动，用单轮命令检查对应电机座、底盘供电和实际接线。

同一 Uno 同时提供关节控制。组合动作通过 `MHS.open_uno` 复用串口，打开时握手停车一次。[机械臂调用示例](../arm/README.md)

底盘位姿可由独立的 IMU 串口与相机提供。[IMU 定位和离线回放](imu/README.md)
