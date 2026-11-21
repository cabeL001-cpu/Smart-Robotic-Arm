# 银立方机器人 · 树莓派部署指南

树莓派运行 Python，通过板载蓝牙与扩展板上的 HC-05 交换命令，树莓派侧绑定为 `/dev/rfcomm0`。经典 Arduino Uno 运行一份同时驱动四向 SR09 避障、四轮和两台 MG995 的固件。距地约 1.5 m 的随车下视相机供机械臂与车体定位共用。配有第二台相机时，车头前摄可独立读取围挡标签。YbImu 使用自己的 USB 串口。

首次装机按第 1–7 节进行。**只验收四轮**时，第 2 节只装 `move/requirements.txt` 并运行底盘预览，第 3 节配对蓝牙并确认 `UNO_PORT`，随后烧录第 4 节的完整草图，按第 5.1 节验证四个 SR09 后点动。只部署语音时，先完成第 2 节的代码复制和虚拟环境创建，再转到第 8 节。实物的串口名、电源规格、轮序、舵机行程和相机标定值需要现场核对。

## 1. 断电接线

准备树莓派及配套电源和系统盘、经典 Uno R3（ATmega328P）及扩展板、四个直流减速电机、两台 MG995、一台下视相机、YbImu、四个 SR09 与五口 IIC 扩展板、4P 线、HC-05、USB 数据线，以及车载主控、底盘和舵机电源。识别车头前方围挡标签时再接前摄；语音模块另需麦克风与扬声器。

| 从哪里接 | 接到哪里 | 接线依据 |
| --- | --- | --- |
| 树莓派板载蓝牙 | 扩展板蓝牙座上的 HC-05 | 无线传底盘和机械臂命令，不占 USB。模块已配置为从模式，名称 `sjtu006`（本批次 `sjtu001`–`sjtu015`），密码 `1234`；树莓派侧绑定为 `/dev/rfcomm0`，固件仍使用 `9600` 波特、8N1、换行结尾的 ASCII 指令 |
| Uno 主控 `G、V、A4、A5` | 五口 IIC 扩展板的 `GND、VCC、SDA、SCL` | 一个端子接主控，另四个端子各接一组 SR09。按 GND 丝印和端子方向核对线序 |
| 前、后、左、右 SR09 | IIC 扩展板任意四个空口 | 以车体为准，前 `0x69`、后 `0x72`、左 `0x6C`、右 `0x6A`。插口并联，不需改址 |
| 树莓派另一 USB 口 | YbImu 的 USB 数据口 | 独立串口，`115200` 波特、8N1。将 IMU 刚性固定在底盘，记录其 +X 相对车头的方向 |
| 另一树莓派 USB 主机口 | 前视 USB 摄像头 | 固定在车头中部，完整拍到围挡标签 |
| 树莓派 USB 主机口 | 随车下视 USB 摄像头 | 本文按 Linux 可读的 V4L2 摄像头部署。支架固定在车上，镜头朝地，棋盘固定在场地上 |
| 电机 1、2、3、4 的两根线 | 扩展板 `M1`、`M2`、`M3`、`M4` 电机座 | 对应的 Uno **方向 / PWM** 分别为 **D2 / D3、D4 / D5、D8 / D6、D12 / D11**。沿用原调参轮序，用单轮点动记录实际车轮位置 |
| 肩部 MG995 信号线 | 扩展板舵机接口 **D13 的 S** | 肩部指靠车体的基座关节 |
| 肘部 MG995 信号线 | Uno 数字排针 **D10**（丝印 `10` 或 `~10`） | 肘部是连接两段臂的关节。扩展板的绿色 `VOUT` 也由 D10 控制，使用舵机期间保持该功率端子空载 |
| 两台 MG995 的电源正线、地线 | 外接舵机稳压电源的 `+`、`−` | 再将该电源 `−` 接 Uno `GND`，让舵机信号共地。用分线接头将正线单独引到外部电源，避免把两个电源的正极并在一起 |
| 底盘电池或匹配的电源 | 扩展板主电源输入 **VIN1**，按实物 `+ / −` | 按扩展板和电机实际额定值选电压，图示 VIN1 上限为 12 V。VIN1 同时给 Uno 提供逻辑供电，树莓派用自己的电源。实车运行时拔除树莓派与 Uno 间的 USB，Uno 逻辑、驱动和舵机由车载供电系统承担 |

先对准 Uno 与扩展板排针，再按丝印识别端子和舵机三根线。MG995 不同批次的线色和行程可能有差异。外置稳压电源应覆盖两台舵机同时动作的峰值电流。底盘电机和舵机的负载电流都走各自供电线路。四个探头共用 Uno 的 IIC 总线。蓝牙只传数据，负载电压和电流仍由车载电源决定。上述接法无需占用树莓派的 40 针 GPIO。[扩展板接口实物图](image-8.png) · [Uno 固件引脚](move/arm_control.ino) · [Raspberry Pi 各型号电源要求](https://www.raspberrypi.com/documentation/computers/getting-started.html)

首次通电时将车轮架空，支撑机械臂并留出摆动空间。先保持电机和舵机的负载电源关闭，只接通树莓派与 Uno 逻辑电源。完成第 4 节烧录和第 5.1 节的停车握手后，再分路接通负载做点动测试。

**HC-05 接法。** 模块直接插在扩展板标注 `5V GND RX TX` 的蓝牙座上，该座与 Uno 的硬件串口 D0/D1 相连，所以固件仍然使用 `Serial`，`move/move.ino` 一行都不用改。板载 USB 留给烧录和负载断电的台架诊断；正因为和板载 USB 串口芯片共用 D0/D1，**烧录和用 USB 串口监视时必须把 HC-05 从座上拔下**，运行时也不要插 USB 数据线，否则两路会抢同一条串口。模块按出厂默认 9600 波特工作；若曾用 AT 命令改过，用 `AT+UART=9600,0,0` 改回。带载前检查轮子起动和舵机动作时的实际电压。

## 2. 准备树莓派系统和代码

**系统。** 对支持 64 位的树莓派（例如 Pi 4、Pi 5），使用 64 位 Raspberry Pi OS Lite 或桌面版。已有可用系统可直接检查。全新系统盘可用 Raspberry Pi Imager 写入，先确认选中的是目标卡，再在自定义设置中设用户名、主机名、网络和 SSH。以下 `operator@robot.local` 是示例，请换成实际用户名和主机名。若 `.local` 无法解析，用树莓派本机 `hostname -I` 显示的局域网 IP。[Raspberry Pi 安装与 SSH 指南](https://www.raspberrypi.com/documentation/computers/getting-started.html)

登录树莓派后，在本机终端或 SSH 会话中检查系统并安装基础工具。

~~~bash
cat /etc/os-release
dpkg --print-architecture
python3 --version
sudo apt update
sudo apt install -y git curl python3-venv python3-pip usbutils v4l-utils libgl1 libgomp1 bluez
sudo usermod -aG dialout,video "$USER"
~~~

64 位系统的架构应显示 `arm64`。加入设备组后退出当前登录，再重新登录，用 `id -nG` 确认有 `dialout` 和 `video`。后续 Python 命令均以普通用户运行。[Arduino Linux 串口权限说明](https://support.arduino.cc/hc/en-us/articles/360016495679-Fix-port-access-on-Linux)

**复制项目，选一种来源。**

- 要部署当前电脑上的代码，在仓库根目录打开 **Windows PowerShell**。将示例的用户名和主机名换成实机值。复制整个 `Agents` 会带上 Uno 草图、限位头文件、Python 包和测试用图。已有同名部署时，先备份实机上的本地改动。

~~~powershell
ssh operator@robot.local "mkdir -p ~/Smart-Robotic-Arm"
scp -r .\Agents operator@robot.local:Smart-Robotic-Arm/
~~~

- 要在一台新树莓派上取远端版本，在**树莓派终端**执行下列命令。树莓派无法联网时也可用 U 盘复制完整 `Agents` 到同一目录，系统包、Python 依赖和可选语音模型仍须事先准备。

~~~bash
git clone https://github.com/cabeL001-cpu/Smart-Robotic-Arm.git "$HOME/Smart-Robotic-Arm"
git -C "$HOME/Smart-Robotic-Arm" rev-parse --short HEAD
~~~

已有 Git 克隆时，用 `git -C "$HOME/Smart-Robotic-Arm" pull --ff-only` 更新。仅用 `scp` 复制了 `Agents` 的设备继续用文件复制更新。

**创建运行环境。** 下文标为 PowerShell 的示例在 Windows 运行，其余部署命令使用树莓派的 Bash。所有 `python -m ...` 命令都从 `Agents` 目录运行。项目和虚拟环境使用普通用户目录，标定与采集数据保存在仓库外。

~~~bash
cd "$HOME/Smart-Robotic-Arm/Agents"
mkdir -p "$HOME/.venvs"
python3 -m venv "$HOME/.venvs/smart-robotic-arm"
source "$HOME/.venvs/smart-robotic-arm/bin/activate"
CFG="$HOME/.config/smart-robotic-arm"
DATA="$HOME/robot-data"
mkdir -p "$CFG" "$DATA"

python -m pip install --upgrade pip
python -m pip install -r move/requirements.txt -r vision/requirements.txt
python -m pip check
python - <<'PY'
import sys
import cv2
import numpy
import serial
from cv2 import aruco

for family in (aruco.DICT_APRILTAG_36h11, aruco.DICT_APRILTAG_36h10):
    aruco.ArucoDetector(aruco.getPredefinedDictionary(family))
print(sys.executable, cv2.__version__, numpy.__version__, serial.__version__)
PY
~~~

最后一行应显示虚拟环境内的 Python 路径和三个库的版本。底盘、机械臂和 IMU 共用 `pyserial==3.5`。视觉要求 `numpy>=1.23` 及 `opencv-python>=4.7,<5`，还需上面的两个 AprilTag 字典和 `ArucoDetector`。只部署底盘、机械臂或 IMU 时，安装 `move/requirements.txt` 即可，跳过视觉导入检查。已有 `~/mx` 等虚拟环境也可用同一检查验证。[Raspberry Pi OS 的 Python 虚拟环境](https://www.raspberrypi.com/documentation/computers/os.html) · [OpenCV Python 安装说明](https://pypi.org/project/opencv-python/)

**离线自检。** 可在插入任何硬件前运行。

~~~bash
python -m move go --ms 250 --dry-run
python -m move.sonar --help
python -m unittest -v arm.test_arm move.test_sonar move.imu.test_imu vision.test_arm vision.test_robot vision.test_scan
python -m vision.scan top --source vision/fixtures/printed-board.jpg --frames 1
python -m vision.arm --source image-7.png --cell 4
~~~

底盘预览输出 `GO 250`，超声波帮助列出 `scan` 与 `read`。实拍棋盘样例应给出 `board_tags_used: 4`。最后一条读取旧模板，图中没有末端标签，预期 `measurement` 为 `null`、`arrived` 为 `false`。机械臂的动作预览需要第 5.2 节的关节标定。

## 3. 配对蓝牙、辨认 IMU 和摄像头

### 3.1 配对 HC-05 并绑定串口

先确认树莓派的蓝牙已启用。HC-05 上电后指示灯快闪，表示处于可配对状态。

~~~bash
sudo systemctl enable --now bluetooth
rfkill unblock bluetooth 2>/dev/null || true
bluetoothctl
~~~

在 `bluetoothctl` 的交互提示符中依次执行。`scan on` 后等待 `sjtu006` 出现在列表里，记下它前面的 MAC 地址（本批次的名称是 `sjtu001` 到 `sjtu015`，以模块上的编号为准）。

~~~text
power on
agent on
scan on
pair <MAC>        # 提示输入密码时填 1234
trust <MAC>
scan off
exit
~~~

配对成功后把模块绑定成串口设备。仓库里的 `bt_link.sh` 会按名字自动查 MAC，先释放旧绑定再重新绑定；也可以手写 MAC。

~~~bash
cd "$HOME/Smart-Robotic-Arm/Agents"
sh bt_link.sh bind sjtu006         # 等价于 sudo rfcomm bind rfcomm0 <MAC> 1
ls -l /dev/rfcomm0

sh bt_link.sh status               # 查看绑定
sh bt_link.sh release              # 需要断开时释放
~~~

日常使用时不需要手动连接：`move`、`arm`、`vision` 每次运行都会在打开端口时自动建立蓝牙连接，程序退出后链路自动断开，模块指示灯回到快闪。**空闲时快闪是正常状态**，不要用灯色判断链路好坏，以 `python -m move stop` 是否打印「Uno 已确认停车。」为准。

树莓派每次重启或 HC-05 断电重连后，`/dev/rfcomm0` 都会消失，需要重新 bind。要开机自动绑定，可新建 `/etc/systemd/system/rfcomm.service`，把 MAC 换成实际地址（本车 `sjtu006` = `00:21:04:32:B4:02`），然后启用它。

~~~ini
[Unit]
Description=Bind HC-05 to /dev/rfcomm0
After=bluetooth.service
Requires=bluetooth.service

[Service]
Type=oneshot
ExecStart=-/usr/bin/rfcomm release rfcomm0
ExecStart=/usr/bin/rfcomm bind rfcomm0 00:21:04:32:B4:02 1
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
~~~

~~~bash
sudo systemctl daemon-reload
sudo systemctl enable --now rfcomm.service
~~~

分步验收、失败处置和记录表见 [HC-05 蓝牙链路配置与验收工作流](BLUETOOTH_WORKFLOW.md)。

### 3.2 辨认 IMU 和摄像头

HC-05 不占用 USB，剩下的 USB 设备只有 YbImu 和摄像头。列出设备，对照描述和 USB 物理位置贴标签。插入摄像头后用 V4L2 找到它实际输出图像的节点。

~~~bash
lsusb
python -m serial.tools.list_ports -v
ls -l /dev/serial/by-id /dev/serial/by-path
v4l2-ctl --list-devices
~~~

把已接设备的配置存入树莓派上的 `$CFG/devices.env`。`UNO_PORT` 填蓝牙串口，`IMU_PORT` 换成实际设备，优先使用唯一的 `/dev/serial/by-id/...` 链接；标识重复时可选 `by-path` 并固定 USB 插口。`CAM` 是下视相机，`FRONT_CAM` 只在接有前摄时填写。用 `ls -l /dev/v4l/by-id/` 核对示例中的设备名。

~~~bash
export UNO_PORT=/dev/rfcomm0
export IMU_PORT=/dev/ttyUSB0
export CAM=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0
export FRONT_CAM=/dev/v4l/by-id/usb-_Webcam_C170-video-index0
~~~

加载设置，确认蓝牙串口、IMU 端口和下视相机。只有下视相机的车也可用它完成第 6 节的末端测量和车体定位。

~~~bash
source "$CFG/devices.env"
ls -lL "$UNO_PORT" "$IMU_PORT"
readlink -f "$UNO_PORT" "$IMU_PORT"
v4l2-ctl -d "$CAM" --list-formats-ext
python - <<'PY'
import os
from contextlib import closing
from vision.camera import Mount, frames

with closing(frames(os.environ["CAM"], Mount())) as stream:
    _, frame = next(stream)
    print("实收宽、高", frame.shape[1], frame.shape[0])
PY
~~~

车头另装前摄时，再单独验证其节点和围挡标签。

~~~bash
v4l2-ctl -d "$FRONT_CAM" --list-formats-ext
python -m vision.scan front --source "$FRONT_CAM" --frames 10
~~~

记下实收分辨率及摄像头支持的其他尺寸。项目把数字编号与 V4L2 设备路径作为实时相机，把文件名作为图片或录像。若实物是 CSI 排线相机，先确认本机 OpenCV 也能通过数字编号取到有效帧。当前采集入口使用 `cv2.VideoCapture`，没有 Picamera2 原生入口。[项目相机采集代码](vision/camera.py)

## 4. 将合并固件烧录到 Uno

`move/move.ino` 是入口，`move_func.ino` 保留轮控参数，`sonar.cpp`、`sonar.h` 负责四向避障，`arm_control.ino` 和 `arm_limits.h` 执行两轴脉宽限位。编译整个 `move` 目录。已完成第 5.2 节标定时先导出限位头文件，否则默认固件仅锁定舵机，仍可验收避障。烧录时四轮架空、负载电源关闭。

**断电拔下 HC-05，再用 USB 将 Uno 接到烧录机。** 板载 USB 串口和 HC-05 共用 D0/D1。关闭串口监视器，让 Arduino CLI 独占烧录口。上传校验通过后拔掉 USB，断电插回 HC-05，再恢复车载供电。

**Windows PowerShell 快速烧录。** 从最新克隆的仓库根目录执行。CLI 首次使用时先运行 `arduino-cli core update-index`、`arduino-cli core install arduino:avr` 和 `arduino-cli lib install Servo`。若 CLI 未加入 PATH，用 `& 'arduino-cli.exe 的完整路径'` 调用。若要启用舵机，先从本车实测标定导出 `move/arm_limits.h`。仅验证四轮和超声波时可使用仓库的默认限位。

~~~powershell
Set-Location .\Agents
arduino-cli board list
$flashPort = Read-Host '输入列表中 Uno 的 USB 端口（例如 COM14）'
arduino-cli compile --fqbn arduino:avr:uno --port $flashPort --upload --verify .\move
~~~

若看到 `not in sync`，先核对实际 USB 端口、HC-05 是否已拔下，以及串口监视器是否已关闭。用可传数据的 USB 线重新连接，重新执行 `board list` 后再上传。CLI 成功退出并通过写入校验后，在树莓派运行 `sh bt_link.sh bind` 和 `python -m move stop` 验证运行链路。

**Arduino IDE。** 打开 `Agents/move/move.ino`，安装支持 `Wire.setWireTimeout` 的 Arduino AVR Boards（已测试 1.8.8）及 Servo 库。选择 Arduino Uno 和烧录用 USB 端口，验证并上传完整草图。[Arduino Uno R3 板卡资料](https://docs.arduino.cc/hardware/uno-rev3/)

**也可在树莓派用 Arduino CLI。** 以下命令用 Arduino 官方安装脚本按系统架构下载 CLI，安装到当前用户的 `~/.local/bin`。已有 CLI 时可从 `arduino-cli version` 开始。编译结果写入临时目录，上传时使用同一目录。烧录期间 Uno 仍要接 USB、HC-05 要拔下，所以上传端口用 `FLASH_PORT`（这次 USB 烧录口），**不能用 `$UNO_PORT` 的蓝牙地址**；`move` 指当前 `Agents` 目录下的草图。上传完成后拔线，运行时继续用 `UNO_PORT`。[CLI 安装](https://docs.arduino.cc/arduino-cli/installation/) · [官方安装脚本](https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh) · [CLI 编译与上传](https://docs.arduino.cc/arduino-cli/getting-started/)

~~~bash
mkdir -p "$HOME/.local/bin"
CLI_SETUP=$(mktemp -d)
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
  -o "$CLI_SETUP/install.sh" && BINDIR="$HOME/.local/bin" sh "$CLI_SETUP/install.sh"
export PATH="$HOME/.local/bin:$PATH"
arduino-cli version
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install Servo
arduino-cli board list
UNO_BUILD=$(mktemp -d)
arduino-cli compile --fqbn arduino:avr:uno --build-path "$UNO_BUILD" move
read -r -p "请输入 board list 中的 Uno USB 烧录端口 " FLASH_PORT
arduino-cli upload --fqbn arduino:avr:uno --port "$FLASH_PORT" --input-dir "$UNO_BUILD" --verify move
~~~

## 5. 分别验收四轮和机械臂

### 5.1 底盘停车与点动

1. 给 Uno、蓝牙模块和四个 SR09 接通车载逻辑供电，保持轮臂负载断电。先握手停车，确认终端显示“Uno 已确认停车。”，再扫描，预期依次显示 `0x69 0x72 0x6C 0x6A`。用 `read --watch`，待约 2 秒预热并完成首轮采样后，拿平板逐个靠近探头，核对方向和毫米值。按 **Ctrl+C** 结束读数。

~~~bash
python -m move stop --port "$UNO_PORT"
python -m move.sonar scan --port "$UNO_PORT"
python -m move.sonar read --port "$UNO_PORT" --watch
~~~

2. 保持四轮离地，接通底盘电源。确认四向都有大于阈值的有效回波后，一次只执行一条命令，观察哪个轮子转、转向是否符合装配、到时是否停下，记录 `M1–M4` 对应的实际轮位。

~~~bash
python -m move m1 --ms 200 --port "$UNO_PORT"
python -m move m2 --ms 200 --port "$UNO_PORT"
python -m move m3 --ms 200 --port "$UNO_PORT"
python -m move m4 --ms 200 --port "$UNO_PORT"
~~~

3. 四路独立点动通过后，架空运行 `python -m move go --ms 250 --port "$UNO_PORT"`，应四轮同时动作、到时停止。再放到平整空地，短时验证 `go`、`left`、`rotate-left` 的实车方向。若单轮转向错误，先断开驱动电源，再核对轮位及电机两线极性。

每次定时动作由 Uno 自行计时。运行中的底盘命令可按 **Ctrl+C** 提前请求停车。`STOP` 控制四轮，舵机在收到关节命令后会继续保持输出。同一时刻只让一个进程占用 Uno 串口，连续控制轮和臂时可用 `MHS.open_uno` 复用一次连接，省去每次重建链路和握手，保持回执与动作对应。蓝牙链路打开时不会复位 Uno；只有走 USB 台架调试时打开串口才可能触发复位。[底盘动作与串口回执](move/README.md) · [共享串口调用示例](arm/README.md)

**验证避障。** 在四轮架空、四向均有回波时，用平板将前方距离降到 300 mm 以内。向前应收到 `ERR OBSTACLE FRONT …`，后方畅通时可后退。再将平板移远、给一个新的短时前进命令，运行中把平板靠近前方，观察四轮主动停车。探头离线或读数过期会得到 `ERR SONAR 方向`，移开障碍后不会自动恢复原动作。初始阈值为 300 mm，落地前依据实测反应与制动距离调整。[四向判定与调试步骤](move/README.md)

### 5.2 两台 MG995 的机械止挡与标定

每轴只能从最大轴位退回 90°，软件默认在两端各留 3°。肩部实物照片记录了机械止挡姿态，斜拍图不能确定控制脉宽。肘部单独标定。按机械臂文档中的步骤，测出两轴的几何角、退回方向和行程内两个已知脉宽点，将标定存为 `$CFG/arm.json`。当关节在止挡处卡住时，先支撑臂、断开舵机电源并卸载，避免强扳或通电顶住机构。[标定表、测量方法与计算公式](arm/README.md)

在要烧录的那份 `Agents` 源码中，先把两轴实测限位导出到固件。第二条可选，用于预览当前配置下的 4 号格。随后按第 4 节重新编译并上传，拔下烧录线，再通过蓝牙调试。改变舵盘安装位置或脉宽标定时需要重新导出、烧录。

~~~bash
python -m arm --config "$CFG/arm.json" --export-limits move/arm_limits.h
python -m arm --config "$CFG/arm.json" --cell 4 --dry-run
~~~

从旧 D7 接线升级时，断电将肘部信号线移至 D10，按第 4 节上传新版完整草图。只改信号线可沿用原实测标定。首次点动将肩角设为当前实测角，选择一个邻近当前肘角且远离端点的肘角，先预览，再运行。旧配置缺少 `angle_at_max_deg` 时需补全，默认固件对 `ARM` 返回 `ERR ARM CALIBRATION`。格心不可达时，重新选择停靠位。`OK ARM` 仅表示两路脉宽已设置，实际到位用第 6 节的相机确认。

~~~bash
read -r -p "实测肩角和肘角（度，以空格分隔） " Q1 Q2
python -m arm --joints "$Q1" "$Q2" --config "$CFG/arm.json" --dry-run
python -m arm --joints "$Q1" "$Q2" --config "$CFG/arm.json" --port "$UNO_PORT"
~~~

## 6. 顶摄定位与前摄墙标识别

顶摄与前摄可同时工作。前摄用第 3.2 节的命令检查 36h11 墙标 1–4。顶摄完成下列标定后，可用 `python -m vision.scan top --source "$CAM" --config "$CFG/mount.json" --frames 10` 查看每个棋盘角标的编号和坐标。[双相机验收与实物角标说明](vision/README.md)

### 6.1 安装棋盘和末端标签

- 九宫格平整固定在场地。实测外边长应为 **200 × 200 mm**。棋盘坐标原点在左下，向右为 +X，向上为 +Y。实物印刷板四角为 5×5 图案（黑框约 15 mm），旧模板为 tag36h11（黑框 20 mm）。两版均按左下 0、右下 1、左上 2、右上 3 映射。图中格号从左下 0 排到右上 8，4 号中心是棋盘坐标 `(100, 100) mm`。用尺检查印后尺寸，仓库里的示意截图本身不保证打印比例。[棋盘布局](image-7.png)
- 末端默认用 **tag36h10 / ID 100**，固定在末端、码面朝上并平行于棋盘。将车停到作业位，用实际镜头检查对焦、曝光和遮挡。要测量末端，画面须同时检出至少两个棋盘角标与末端标签。约 1.5 m 高度下，应以实收画面中能稳定识别小标签的分辨率进行标定。
- 用 `nano "$CFG/mount.json"` 保存下面的配置。`image_size` 改为摄像头确实支持的宽、高。`height_mm` 从镜头中心量到棋盘平面，`tip_height_mm` 从棋盘面量到末端标签面。`tip_offset_mm` 是标签中心到实际指尖沿标签自身“向右、向上”的毫米偏移量。`0` 只适用于相应的高度差或偏移确实为零时。

~~~json
{
  "image_size": [1280, 720],
  "height_mm": 1500,
  "tip_height_mm": 0,
  "tip_offset_mm": [0, 0],
  "tip_family": "36h10",
  "tip_id": 100
}
~~~

近乎垂直安装时程序默认铅垂落点在图像中心。存在倾斜时，在同分辨率的去畸变图像中量出镜头铅垂线落在棋盘平面的像素，填入 `nadir_px`。使用镜头标定时，按 OpenCV 标定流程用多姿态棋盘图测得 `camera_matrix`（3×3）和 `distortion`（畸变系数列表），一并写入配置。调整焦距、裁剪、分辨率、相机支架或安装姿态后应重新标定。[OpenCV 相机标定](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)

### 6.2 先测量，再标定机械臂和车体

以下是三个独立的操作。持续打印的命令用 **Ctrl+C** 结束，再打开下一个相机程序。

1. **只测末端。** 保持底盘静止。若手动放置指尖，先支撑臂并断开舵机负载电源。运行下面的命令，将指尖对准 4 号格心，观察 `measurement.tip_mm` 和 `error_mm`。移开指尖，误差应随之变化。连续有效观测误差严格小于 **6.7 mm** 达 **2 秒** 才有 `arrived: true`。标签丢失或相邻帧间隔超过 0.5 秒会重新计时。用实物尺再核对一次指尖到中心的距离。

~~~bash
python -m vision.arm --source "$CAM" --cell 4 --config "$CFG/mount.json"
~~~

2. **标定肩轴在画面中的位置。** 停在已知对齐工位，量肩轴相对**棋盘左下角**的 X、Y（毫米），量肩零位相对棋盘 +X 的逆时针角度（度）。将下例 `-150 100 0` 换成实测值。程序在第一帧有足够棋盘角标的图像上保存 `rig.json` 后退出。此时舵机可保持断电。

~~~bash
python -m vision.align --source "$CAM" --config "$CFG/mount.json" \
  --base -150 100 0 --save-mount "$CFG/rig.json"
~~~

3. **标定车心与车头。** 场地坐标原点在场地图左下，朝 +X 为 0°，逆时针为正。默认场地 2 × 2 m，棋盘左下在场地 `(0.9, 0.9) m`，棋盘 +X 与场地 +X 同向。将车心停在场内一个**独立量过**的点，量车头方向，且让摄像头看见固定参照。下面 `1 1 0` 的单位依次是米、米、度，表示车心在 `(1, 1) m` 且车头朝 +X。读取上一步的 `rig.json`，生成的 `vehicle.json` 将同时保留机械臂和车体的安装标定。若只部署车体定位，可跳过肩轴标定，直接把下面第一条命令的输入配置换成 `mount.json`。

~~~bash
python -m vision.robot --source "$CAM" --config "$CFG/rig.json" \
  --calibrate 1 1 0 --save-mount "$CFG/vehicle.json"
python -m vision.robot --source "$CAM" --config "$CFG/vehicle.json" --cell 4
~~~

将车移到第二个实测点，改变朝向，对比 `pose.x_m`、`pose.y_m`、`pose.yaw_rad` 与尺量结果。输出角度为**弧度**，逆时针 90° 约为 `+1.57`。`--cell 4` 会同时输出同一帧的 `arm` 观测，可用于顶摄联合测量。场地摆法变了，用 `--field` 指向实测的场地 JSON，标定车体、读取车体位姿及后续融合时都使用同一文件。下视视野看不到固定参照时，`pose` 会为 `null`。需要扩大覆盖时，现有代码可读额外的已测量地面标签。墙面标签仅在实际入镜且有内参及标签安装高度时可用。[场地布局](image-2.png) · [场地配置字段与示例](vision/README.md)

**闭环驱动机械臂。** 结束占用摄像头和 Uno 的其他命令，停稳底盘，确认当前位置的目标格在机械臂可达范围内。下面的命令同时使用舵机和相机标定。若只部署机械臂，可在 `--config` 处直接使用第 6.2 节生成的 `rig.json`。

~~~bash
python -m vision.align --source "$CAM" --config "$CFG/vehicle.json" \
  --arm-config "$CFG/arm.json" --cell 4 --port "$UNO_PORT"
~~~

该命令会实际移动两个关节。观察终端的“连续到位两秒”及末帧误差，随后可换一个格心复核。在车体姿态已标定后，同时看车体和末端用 `vision.robot --cell`。需要调整关节时，把摄像头交给 `vision.align`。这两个命令应分时运行。

## 7. IMU、视觉校正与记录回放

**独立验收。** 将车放在已知点，车头朝已知方向，开始后至少静止两秒，让程序估计零偏。`--origin` 的单位依次为米、米、度。`--mount-yaw` 是从车头到 IMU 的 +X 轴逆时针转过的角度。下例适用于车在原点朝 +X，且 IMU +X 与车头同向。

~~~bash
python -m move.imu --port "$IMU_PORT" --origin 0 0 0 --mount-yaw 0 --stationary
~~~

看到 JSON 连续输出，观察 `x_m`、`y_m`、`yaw_rad`、`vx_mps`、`vy_mps`。`--stationary` 会持续施加零速约束，适合在台架上检查初始航向和数据输出。测量实际运动时去掉 `--stationary` 重新启动，重复静止校准，再缓慢转车约 90° 核对航向、沿已知距离移动并比较位移。如需做传感器水平或磁力计校准，使用随硬件提供的 YbImu 上位机完成，退出上位机后再运行本程序。[IMU 协议与校准说明](move/imu/README.md)

**与同一摄像头联合记录。** 先结束独立验收程序。在树莓派同一 Bash 会话执行。将 `--origin` 和 `--mount-yaw` 换成当前实际值，保持静止完成初始化。`RUN` 每次生成新目录，`--record` 会拒绝覆盖已有原始文件。记下终端打印的目录名。

~~~bash
RUN=$(mktemp -d "$DATA/run-XXXXXXXX")
echo "$RUN"
python -m vision.robot --source "$CAM" --config "$CFG/vehicle.json" --cell 4 \
  | tee "$RUN/vision.jsonl" \
  | python -m move.imu --port "$IMU_PORT" --origin 0 0 0 --mount-yaw 0 \
      --vision-stdin --record "$RUN/imu-raw.csv" \
  | tee "$RUN/fused.jsonl"
~~~

让相机看到固定棋盘，移动时保持 IMU 连续采样。`vision_fixed: true` 表示该次 IMU 更新接受了视觉校正。被遮挡时会暂时依靠惯性推算，位置误差会随时间累积。断电搬车后，从实测位姿重新启动，或等可见的固定地标提供绝对位置。视觉与 IMU 进程须在同一台树莓派上运行，以共享单调时钟。

完成一段运动后按 **Ctrl+C**。下次可用同一目录的**原始 IMU 样本**和**同期视觉记录**回放，不需要接任何设备。在新会话中先将 `RUN` 设为录制时打印的目录路径。使用与录制相同的初始位姿和安装角。`fused.jsonl` 保存当时的结果，`imu-raw.csv` 才是回放输入。

~~~bash
python -m move.imu --replay "$RUN/imu-raw.csv" --fixes "$RUN/vision.jsonl" \
  --origin 0 0 0 --mount-yaw 0
~~~

### 7.1 运行 A/B/C/D 和坐标导航

部署支持 `NAV` 指令的本版完整 Uno 固件。分别保存 `$CFG/navigation/field.json`、`front.json` 和 `nav.json`，测量墙标地图、前摄内外参、起点与六向运动速率。四向净空读数正常后结束各串口与相机的独立读取程序。导航会话会独占这三台设备。[逐项标定及验收方法](move/CALIBRATION.md)

~~~bash
source "$CFG/devices.env"
python -m move.navigation --config "$CFG/navigation/nav.json" --dock A B C D
~~~

同一命令的四段行程共用启动原点。遇障或探头缺测时 Uno 锁停，移开障碍后仍保持停车。人工确认四面有效且畅通后，`python -m move.navigation --reset-lock` 解除锁存。再次出发时按当前起姿重新建系。[任意坐标与异常接口](API.md)

## 8. 可选语音部署

### 8.1 麦克风、扬声器与离线识别

在项目虚拟环境安装语音依赖。桌面系统可用 `pw-play` 播放，Lite 系统可选 ALSA 输出。USB 扬声器或 HDMI 输出均以本机枚举结果为准。

~~~bash
sudo apt install -y alsa-utils ffmpeg espeak-ng unzip
python -m pip install -r listen_speak/requirements.txt
python -m pip check
arecord -l
arecord -L
aplay -l
aplay -L
~~~

`listen_speak/config.py` 中 `MIC_DEVICE` 默认是 `plughw:CARD=WEBCAM,DEV=0`。若与实际录音设备不同，在树莓派副本填写实际名称。用同一设备录 3 秒，回听应能听清人声。程序检测到 `pw-play` 时使用 PipeWire 默认输出。使用 ALSA 时将 `PLAY_DEVICE` 设成 `aplay -L` 列出的实际设备。

~~~bash
arecord -D 'plughw:CARD=WEBCAM,DEV=0' -f S16_LE -r 16000 -c 1 -d 3 "$DATA/mic-test.wav"
if command -v pw-play >/dev/null; then
  pw-play "$DATA/mic-test.wav"
else
  aplay -D default "$DATA/mic-test.wav"
fi
~~~

离线中文转写使用 Vosk。下面将模型放入代码默认读取的位置，`unzip -n` 保留已有文件。目录应直接包含 `am`、`conf` 等子目录。若模型放在别处，修改 `VOSK_MODEL_DIR`。[Vosk 官方模型](https://alphacephei.com/vosk/models)

~~~bash
mkdir -p "$HOME/Desktop/mxpidemo/sound"
curl -fL -o "$DATA/vosk-model-small-cn-0.22.zip" \
  https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip
unzip -n "$DATA/vosk-model-small-cn-0.22.zip" -d "$HOME/Desktop/mxpidemo/sound"
test -d "$HOME/Desktop/mxpidemo/sound/vosk-model-small-cn-0.22/am"
python -c 'from listen_speak import recognize; print(recognize())'
~~~

对着麦克风说“请到九宫格四号位置”，终端应显示识别到的中文。空字符串表示这次没有得到有效语音。此步骤可断网执行。

### 8.2 离线播报与在线口令解析

基础离线播报可用已安装的 `espeak-ng`。要使用 Piper 中文音色，将同一版本的模型及其配置放入以下路径。断网部署时可事先在电脑下载这两个文件后一并复制。[Piper 华严中文模型及配置](https://huggingface.co/rhasspy/piper-voices/tree/main/zh/zh_CN/huayan/medium)

~~~bash
mkdir -p listen_speak/assets/piper
VOICE_URL=https://huggingface.co/rhasspy/piper-voices/resolve/main/zh/zh_CN/huayan/medium
for name in zh_CN-huayan-medium.onnx zh_CN-huayan-medium.onnx.json; do
  curl -fL -o "listen_speak/assets/piper/$name" "$VOICE_URL/$name"
done
python -c 'from listen_speak import speak; speak("机械臂语音测试", offline=True)'
~~~

完整 `listen()` 会录音、用 Vosk 本地转写，再向配置的在线模型发送**转写文本**分类。检查 `listen_speak/config.py` 的 `LLM_BASE_URL` 和账户可用的 `LLM_MODEL`。在交互式终端执行下面的命令，按提示输入该服务的模型名和密钥，密钥只存在本次 Python 进程中。单独设置系统环境变量不会自动填入当前代码的 `LLM_API_KEY`。

~~~bash
python -c 'from getpass import getpass
from listen_speak import config, listen
config.LLM_MODEL = input("模型名（回车沿用配置） ").strip() or config.LLM_MODEL
config.LLM_API_KEY = getpass("API Key ")
print(listen())'
~~~

说“请到九宫格四号位置”，期望返回 `('4', 'arrive')`，空指令等情况返回 `(None, None)`。当前语音分类支持格号 0–8。A/B/C/D 可从命令行指定，或由调用方将 `('A','arrive')` 和导航会话传给 `MHS.dispatch`。[语音接口](listen_speak/__init__.py) · [系统接口手册](API.md)

## 9. 下次开机与故障定位

本版在开发机上通过了第 2 节的模块测试、四向 SR09 虚拟总线测试和 Uno AVR 固件编译。仓库示例图可验证四个棋盘角标及标定文件的传递，实际转向、供电稳定性与格心误差需在实车上验收。

重新登录后，先恢复工作目录、虚拟环境和实机设备配置。相机支架、棋盘和舵机安装未变时可沿用标定文件。每次启动 IMU 都需要新的静止校准。

~~~bash
cd "$HOME/Smart-Robotic-Arm/Agents"
source "$HOME/.venvs/smart-robotic-arm/bin/activate"
CFG="$HOME/.config/smart-robotic-arm"
DATA="$HOME/robot-data"
source "$CFG/devices.env"
sh bt_link.sh bind           # 重启或模块重连后要恢复 /dev/rfcomm0
python -m move stop --port "$UNO_PORT"
~~~

| 现场现象 | 按顺序检查 |
| --- | --- |
| `No module named move`、`serial` 或缺少 `ArucoDetector` | `pwd` 应为项目的 `Agents`，`which python` 应在虚拟环境内。运行第 2 节的安装与导入检查。用 `python -m pip list` 检查 OpenCV。同一环境只留一种提供 `cv2` 的发行包 |
| `/dev/rfcomm0` 不存在 | 树莓派重启或 HC-05 掉电重连后绑定会失效。在 `Agents` 运行 `sh bt_link.sh bind` 重新绑定。蓝牙服务没起来时先 `sudo systemctl enable --now bluetooth`，用 `rfkill list` 确认没被软关闭 |
| 扫描不到 `sjtu006`、配对失败 | 确认模块在可配对状态（指示灯快闪），没有别的设备已经连着它，树莓派与模块距离足够近。密码为 `1234`。用 `bluetoothctl devices` 核对名字和 MAC |
| Uno 或 IMU 报权限不足、串口正忙 | 用 `id -nG`、`ls -lL "$UNO_PORT" "$IMU_PORT"` 检查 `dialout` 权限，重新登录使组生效。`/dev/rfcomm0` 由 root 创建，必要时 `sudo chown $USER /dev/rfcomm0`。同一时刻只让一个进程占用串口，关闭 IDE 监视器、minicom 或此前仍在运行的进程。HC-05 断电时打开节点可能先卡住约半分钟再报错，属于内核在等 RFCOMM 连接 |
| 蓝牙链路时断时续、Uno 不确认停车 | 查模块供电（扩展板 5V 座）、距离和 2.4 GHz 干扰，重新 `sh bt_link.sh bind`。Uno 按本机时钟计时，链路中断时当前动作仍会到期自动停车，不会一直转 |
| `Uno 未确认停车`，或 USB 上传失败、找不到板 | 运行时查蓝牙端点、Uno 独立供电和 9600 波特协议；打开蓝牙链路后开头几个字节可能丢失，重试即可。上传时另查 `FLASH_PORT`、数据线、`arduino:avr:uno` 板型和串口占用，烧录前确认 HC-05 已从蓝牙座拔下，避免把 `UNO_PORT` 当作 USB 烧录口。若有 `OK STOP` 但轮子不动，查底盘电源和对应 M 插座 |
| `ERR OBSTACLE` 或 `ERR SONAR` | 前者包含方向、毫米值，确认障碍和阈值。后者查对应方向是否预热、探头标签、4P 线序与供电，运行 `move.sonar scan` 和 `read --watch`，用平板验证回波 |
| 舵机拒绝动作、抖动、一动就复位或卡住 | `ERR ARM CALIBRATION` 表示固件还缺实测限位头。`ERR ARM` 检查两轴是否与同次标定匹配。支撑臂、断开舵机负载电源后检查机构、稳压电源带载能力、共地和 `S` 针，再无负载检查 `zero_us` 与斜率符号；卡到机械限位时缩小允许脉宽 |
| 相机打不开、分辨率报错 | 关闭其余取流进程，重跑第 3 节的实际读帧测试。选有采集格式的 `/dev/videoN`，在 `--source` 传数字 `N`，用支持的尺寸更新标定 |
| `measurement: null`、`arm: null` 或 `pose: null` | 末端测量检查角标数量、末端标签家族/ID、焦距和遮挡。车体位姿检查固定参照是否入镜及 `vehicle.json`、`--field` 是否匹配。位置有固定偏差时复核打印比例、指尖偏移、镜头和标签高度 |
| IMU 三秒无有效数据、静止校准失败或位移漂移 | 核对 IMU 独立端口和 115200 波特，厂商输出须同时有原始传感器帧 `0x04` 与姿态帧 `0x16`（四元数）或 `0x26`（欧拉角）。车辆静止重启，核对安装角与初始点，再检查是否接受了视觉校正 |
| 语音无声、转写为空或接口报错 | 先回放 3 秒录音，检查当前用户的音频设备权限、输入和默认输出。再查 Vosk 目录及 Piper 的两个同名文件。单独转写成功后，再核对模型名、密钥和网络 |

结束实车测试时在运行终端按 **Ctrl+C**，确认底盘停止，支撑机械臂后关闭两路负载电源。标定保存在 `$CFG`，原始记录和视觉记录保存在 `$DATA`。树莓派用 `sudo poweroff` 正常关机，待系统关机完成再切断树莓派电源。
