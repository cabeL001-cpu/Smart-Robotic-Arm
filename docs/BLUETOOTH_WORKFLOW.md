# HC-05 蓝牙链路配置与验收工作流

树莓派通过扩展板蓝牙座上的 HC-05 向 Uno 下发底盘、机械臂和视觉闭环动作。单独测量视觉、IMU 和识别语音时可分别运行各模块。Uno 固件使用硬件串口 `Serial`。

链路全貌：

```text
树莓派板载蓝牙 ──SPP──> HC-05（从模式，sjtu013，PIN 1234） ──D0/D1──> Uno 固件
                           扩展板 5V GND RX TX 座
```

配套文档：[部署指南](DEPLOY_RASPBERRY_PI.md) · [底盘命令与回执](move/README.md) · [机械臂](arm/README.md) · [视觉闭环](vision/README.md)

## 0. 验收目标

一次完整的配置验收以「全部必检项通过」为准。必检项共 6 组：

| 编号 | 组别 | 通过标准 |
| --- | --- | --- |
| A | 模块预检 | 名称为 `sjtu013`、角色为从模式、UART 为 9600 |
| B | 配对与绑定 | `Paired: yes`、`Trusted: yes`，`/dev/rfcomm0` 存在 |
| C | 链路往返 | 10 次 `STOP` 全部收到 `OK STOP` |
| D | 烧录 | 拔模块烧录成功，插回后重新绑定成功 |
| E | 功能 | 停车、四轮点动、机械臂、视觉闭环均通过 |
| F | 稳定性 | 断链后已接受的动作到期停车，模块重连后可再次握手 |

任何一项失败，先按第 8 节处置，不要进入演示。

## 1. 前置条件

- 树莓派已按[部署指南第 2 节](DEPLOY_RASPBERRY_PI.md)装好系统、虚拟环境和 `bluez`，当前用户在 `dialout` 组（改组后需重新登录）。
- Uno 已插好扩展板，**HC-05 暂不插上**，先完成一次固件烧录（见第 5 节）。
- 底盘架空，负载电源可随时断开。
- 知道本机模块编号：本批次为 `sjtu001`–`sjtu015`，本次使用 `sjtu013`。

以下命令均在树莓派的 `src` 目录、已激活的虚拟环境中执行。实机配置沿用部署指南，先在当前终端设置 `CFG="$HOME/.config/smart-robotic-arm"`。首次配对可直接从第 3 节开始；已有绑定时从停车握手开始。

## 2. A · 模块预检

这一步只在模块被人动过 AT 配置、或配对后无法通信时做；正常情况可跳到第 3 节。

把 HC-05 从扩展板取下，按住模块上的按钮再插入 USB-TTL（或按住按钮上电），进入 AT 模式。用 sscom 或 minicom 连接，**HC-05 的 AT 波特率通常为 38400 8N1**；若无响应再试 9600。

SSCOM 是大虾丁丁出品的免费 Windows 串口调试助手，课程包里通常自带，绿色免安装，官网见软件底部的 `www.daxia.com`。不想用它也可以：Arduino IDE 的串口监视器（波特率 38400、行尾选 **Both NL & CR**）、PuTTY，或树莓派上的 `picocom -b 38400 /dev/ttyUSB0` 都能发同样的命令。注意 **AT 命令只能在 USB-TTL 的 AT 模式下发送**；经 `/dev/rfcomm0` 透传时，`AT+...` 会被当成普通数据发给 Uno。

~~~text
AT            -> OK
AT+NAME?      -> +NAME:sjtu013      OK
AT+ROLE?      -> +ROLE:0            OK    # 0 = 从模式
AT+UART?      -> +UART:9600,0,0     OK    # 透传波特率必须是 9600
~~~

任一项不符时按下列命令改正，再 `AT+RESET`，然后把模块插回扩展板：

~~~text
AT+NAME=sjtu013
AT+ROLE=0
AT+UART=9600,0,0
AT+RESET
~~~

**判定：** 三条查询都返回期望值。指示灯行为因批次而异（常见为未连接快闪、连接后双闪），**以 bluetoothctl 状态为准，不看灯**。

## 3. B · 配对与绑定

### 3.1 打开树莓派蓝牙

~~~bash
sudo systemctl enable --now bluetooth
rfkill list bluetooth          # 应为 Soft blocked: no
bluetoothctl
~~~

### 3.2 扫描并配对

在 `bluetoothctl` 提示符中执行：

~~~text
power on
agent on
default-agent
scan on
~~~

等待出现类似 `[NEW] Device AA:BB:CC:DD:EE:FF sjtu013` 的行，记下 MAC，然后：

> 本车实测：在用模块 `sjtu013` = `00:21:09:00:12:EC`。`sjtu006` = `00:21:04:32:B4:02` 的 TXD 方向损坏（只能收、不能发），绑定它会出现「Uno 未确认停车」，不要使用。换模块时以 `bluetoothctl devices` 为准，下文的 `AA:BB:CC:DD:EE:FF` 换成实际地址。

~~~text
pair AA:BB:CC:DD:EE:FF      # 提示输入 PIN 时填 1234
trust AA:BB:CC:DD:EE:FF
scan off
exit
~~~

**判定：** 执行下列命令，输出中 `Paired: yes`、`Trusted: yes`。

~~~bash
bluetoothctl info AA:BB:CC:DD:EE:FF
~~~

### 3.3 绑定成串口

~~~bash
sh bt_link.sh bind sjtu013
ls -l /dev/rfcomm0
rfcomm show rfcomm0
~~~

**判定：** `/dev/rfcomm0` 存在且当前用户可读写；`rfcomm show` 显示设备名、channel 1。

树莓派重启、HC-05 掉电或走出范围后，绑定都会失效，重跑 `sh bt_link.sh bind` 即可。需要开机自动绑定，见[部署指南第 3.1 节](DEPLOY_RASPBERRY_PI.md)的 systemd 单元。

### 3.4 日常连接与断开

通过 `rfcomm bind` 登记后，程序打开 `/dev/rfcomm0` 时才会建立连接。底盘控制、关节点动和 `vision.align`、`vision.servo` 闭环驱动都会打开串口，用完关闭。`vision.scan`、`vision.arm` 和 `vision.robot` 可单独采集画面。HC-05 空闲时可能快闪，判断链路用实际停车回执。

~~~text
配对（一次，每台树莓派各做一遍）
  └─ rfcomm bind：在内核里登记 /dev/rfcomm0 指向模块，不建立连接
       └─ 打开 /dev/rfcomm0：此时才真正连上蓝牙，灯变双闪
            └─ 程序退出、端口关闭：挂断，灯回快闪
~~~

每次要控制小车，按顺序执行：

~~~bash
cd ~/Smart-Robotic-Arm/src
source ~/.venvs/smart-robotic-arm/bin/activate   # 若虚拟环境在仓库根目录，改用 .venv/bin/activate
source ~/.config/smart-robotic-arm/devices.env   # UNO_PORT=/dev/rfcomm0
sh bt_link.sh bind                               # 仅树莓派重启或 /dev/rfcomm0 消失后需要
python -m move stop                              # 必须看到「Uno 已确认停车。」
~~~

| 命令 | 作用 | 什么时候用 |
| --- | --- | --- |
| `sh bt_link.sh bind [MAC或名称]` | 登记 `/dev/rfcomm0`，不连接；省略时按默认名称 `sjtu013` 查 MAC | 树莓派重启后、节点消失、连接失败后 |
| `sh bt_link.sh release` | 删除登记，清掉半断开状态 | 重绑前，或要换模块时 |
| `sh bt_link.sh status` | 查看绑定和节点属主 | 排查时 |
| `python -m move stop` | 打开端口 → 连接 → 发 `STOP` → 等回执 → 关闭 | 每次使用前的连通性检查 |
| `sudo rfcomm connect rfcomm0 <MAC> 1` | 主动连接并保持，前台阻塞 | 只在诊断连接失败时用 |

连不上时用 `rfcomm connect` 看具体错误。成功会打印 `Connected /dev/rfcomm0 to ... on channel 1`；Ctrl-C 断开后该临时节点也会消失，需要重新 bind。

| 报错 | 含义 | 处理 |
| --- | --- | --- |
| `Host is down` / `No route to host` | 模块不在线、在 AT 模式，或距离太远 | 查模块供电与指示灯，靠近后重试 |
| `Connection refused` | 模块在线但不接受连接，常见于链路密钥失效 | 重新配对：`bluetoothctl remove <MAC>` → 模块断电重上 → `scan on` → `pair`（PIN 1234）→ `trust` |
| `Address already in use` | 已有绑定 | 先 `sh bt_link.sh release` |
| `Permission denied` | 当前用户不在 `dialout` 组 | `sudo usermod -aG dialout $USER` 后重新登录 |

## 4. C · 链路往返验收

先不碰电机，只验证字节能不能来回。直接打开串口发 `STOP`：

~~~python
import time
import serial

link = serial.Serial("/dev/rfcomm0", 9600, timeout=1, exclusive=True)
link.reset_input_buffer()
round_trips = []
for _ in range(10):
    start = time.monotonic()
    link.write(b"STOP\n")
    reply = link.readline().strip()
    assert reply == b"OK STOP", reply
    round_trips.append((time.monotonic() - start) * 1000)
link.close()

round_trips.sort()
print(f"10/10 OK STOP，中位 {round_trips[5]:.0f} ms，最大 {max(round_trips):.0f} ms")
~~~

**判定：** 10 次全部收到 `OK STOP`；中位数一般几十毫秒，最大值低于 500 ms 均可接受。出现超时先查模块是否插好、是否仍处于连接状态。

手工用 `cat` / `printf` 测试时要先关掉回显：`/dev/rfcomm0` 是 tty，默认开启 ECHO，Uno 的回执会被回显成新的输入送到 Uno。`OK STOP` 带空格，会被固件解析成非法时长，于是回 `ERR DURATION`；这一行同样带空格，被回显后又触发一次，形成无限循环。测试前执行：

~~~bash
stty -F /dev/rfcomm0 raw -echo
~~~

上面的 Python 脚本不受影响，pyserial 打开端口时会自动设为 raw 模式。

这一步同时验证了固件在跑：如果没有任何回复，优先怀疑固件没烧录或模块没插在蓝牙座。

## 5. D · 固件烧录与链路恢复

**HC-05 和板载 USB 串口芯片共用 D0/D1，烧录前必须先拔下模块。**

1. 断开底盘和舵机负载电源，车轮架空。
2. 从扩展板蓝牙座拔下 HC-05。
3. 用 USB 线把 Uno 接到烧录机，按 `arduino-cli board list` 显示的实际 USB 端口上传完整的 `move` 草图并校验。Windows 端口形如 `COM14`，树莓派上常见 `/dev/ttyACM0`。[Windows 和树莓派烧录步骤](DEPLOY_RASPBERRY_PI.md#4-将合并固件烧录到-uno)
4. 关闭串口监视器，拔掉 USB。
5. 把 HC-05 插回蓝牙座，接上 VIN1 逻辑电源。
6. 重新绑定并做一次停车握手：

~~~bash
sh bt_link.sh bind
python -m move stop
~~~

**判定：** 打印「Uno 已确认停车。」。若报 `/dev/rfcomm0 不存在`，回到第 3.3 节。

## 6. E · 功能验收

### 6.1 底盘

~~~bash
python -m move m1 --ms 200    # 依次换成 m2、m3、m4
python -m move go --ms 250
python -m move stop
~~~

**判定：** 单轮点动时只有对应车轮转动约 200 ms；`go` 四轮同时前进、到时自动停止并回 `DONE`；`stop` 立即停车。记录 `M1–M4` 与实物轮位的对应关系。

### 6.2 机械臂

先用本车的 `arm.json` 导出限位，烧录含该限位的固件。支撑连杆，选择靠近当前姿态、远离止挡的实测角度。默认固件收到关节指令会返回 `ERR ARM CALIBRATION`。[两轴标定与点动](arm/README.md)

~~~bash
read -r -p "输入行程内的肩角和肘角（度，以空格分隔） " Q1 Q2
python -m arm --joints "$Q1" "$Q2" --config "$CFG/arm.json" --dry-run
python -m arm --joints "$Q1" "$Q2" --config "$CFG/arm.json"
~~~

**判定：** 每次收到 `OK ARM`，舵机转到目标且不抖、不撞限位。

### 6.3 视觉闭环

~~~bash
python -m vision.align --source 0 --config "$CFG/rig.json" \
  --arm-config "$CFG/arm.json" --cell 4
~~~

**判定：** 程序连续两秒看到末端误差小于 6.7 mm 后返回到位；单帧测量误差在标定允许范围内。

### 6.4 共用连接

连续控制车轮和机械臂时可复用一次链路。把车停在已确认可达 4 号格的标定工位，在同一连接中停车、查询净空，再发送关节指令。

~~~python
from pathlib import Path
import MHS
from arm import ArmConfig
from move.sonar import read

cfg = ArmConfig.load(Path.home() / ".config/smart-robotic-arm/arm.json")
with MHS.open_uno() as link:
    MHS.move("stop", link=link)
    print(read(link)["front"])
    MHS.arm("4", config=cfg, link=link)
~~~

**判定：** 三步在同一连接上按序完成，没有 `ERR BUSY` 或超时。

### 6.5 IMU 并行

IMU 走独立 USB 串口，蓝牙链路启用后应不受影响：

~~~bash
python -m move.imu --port "$IMU_PORT"
~~~

**判定：** 持续输出 JSON 行，静止时位置基本不漂。

## 7. F · 稳定性与断链

1. **距离与遮挡。** 让小车在演示场地内走一圈，链路全程可用。2.4 GHz 干扰大时可把模块远离树莓派天线，或换一个 Wi-Fi 信道。
2. **运动中掉线。** 架空执行 `python -m move go --ms 3000`，中途拔掉 HC-05 的 5V。车轮应在 3 秒到期时停止 —— Uno 自己计时，不依赖链路，这是防失控的兜底。
3. **重连恢复。** 给模块重新上电，等它重连，执行：

~~~bash
sh bt_link.sh release
sh bt_link.sh bind sjtu013
python -m move stop
~~~

**判定：** 掉线时按时停车；重连后能再次握手，不残留旧连接。

4. **连续握手。** 重复第 4 节的 10 次往返脚本 5 轮，全部通过、无进程报「串口正忙」。

## 8. 失败处置

| 现象 | 先查 | 处理 |
| --- | --- | --- |
| `scan on` 看不到 `sjtu013` | 模块是否上电、是否已被别的设备连接 | 断开其他设备，靠近后重扫；必要时进入 AT 模式核对名字 |
| `pair` 失败或没有 PIN 提示 | `agent on` 是否执行、PIN 是否 1234 | `remove` 后重新 `scan on` → `pair` |
| `/dev/rfcomm0` 不存在 | 是否配对成功、是否重启过 | `sh bt_link.sh bind sjtu013`；仍失败先 `bluetoothctl info` 确认 `Paired: yes` |
| `Uno 未确认停车`，`rfcomm` 显示绑定 `00:21:04:32:B4:02` | 绑到了 sjtu006（TXD 方向坏） | `sh bt_link.sh bind sjtu013`，再用 `python -m move stop` 验证 |
| 打开串口卡住或报错 | 模块是否在线、是否有进程占用 | 关闭 minicom/串口监视器；模块断电时打开可能等半分钟才报错 |
| 权限不足 | 是否在 `dialout` 组、节点属主 | `sudo chown $USER /dev/rfcomm0`，或重新登录使组生效 |
| 收到乱码或 `ERR LINE` | 波特率是否 9600 | 回第 2 节用 `AT+UART?` 核对 |
| 停不下来的 `ERR DURATION` | 是否用 `cat` / `printf` 直接测、tty ECHO 是否开启 | `stty -F /dev/rfcomm0 raw -echo` 后重测，或改用 Python |
| 指示灯一直快闪 | 空闲时这是正常状态，链路只随程序打开而建立 | 以 `python -m move stop` 是否打印「Uno 已确认停车。」为准 |
| `rfcomm connect` 报 `Host is down` | 模块没电、在 AT 模式或太远 | 查供电和灯色，靠近后重试 |
| `rfcomm connect` 报 `Connection refused` | 链路密钥失效 | 重新配对：`remove` → 模块断电重上 → `pair`（1234）→ `trust` |
| 有 `OK STOP` 但轮子不动 | 点动命令是否返回 `ERR OBSTACLE` 或 `ERR SONAR` | 用 `python -m move.sonar read` 核对方向和净空；若动作已收到 `DONE` 但车轮未转，断电检查底盘电源和对应 M 插座 |
| 运行中偶发 `Uno 回执超时`，尤其底盘电机启动后 | `vcgencmd get_throttled`、`rfcomm -a`、`hcitool con` | 先看 5V 供电：`0x50005`（bit0+bit16）表示正在/曾经欠压，换 5V/3A 电源或给 HC-05 独立 5V 并共地；欠压会带着 Uno/HC-05 一起复位或掉链 |
| `避障恢复失败，请手动发送 AVOID ON` | 同一次掉链已让 `AVOID ON` 回执丢失 | 链路恢复后执行 `python3 -m move --avoid on`；重定位已自动重连并重试一次，仍失败才提示手动 |

## 9. 回滚到 USB

演示现场不能立刻修好蓝牙时，可以临时切回有线，其余命令不变：

1. 断电，从蓝牙座拔下 HC-05。
2. USB 线连接树莓派与 Uno（此时蓝牙不可用，两者不能同时接）。
3. 用环境变量覆盖端口：

~~~bash
UNO_PORT=/dev/ttyACM0 python -m move stop
UNO_PORT=/dev/ttyACM0 python -m move.sonar read
~~~

排查完再按第 3、5 节恢复蓝牙。

## 10. 每次上电的最小启动序列

保存为开机卡或贴在车上的便签：

~~~bash
cd ~/Smart-Robotic-Arm/src
source ~/.venvs/smart-robotic-arm/bin/activate      # 若虚拟环境在仓库根目录，改用 .venv/bin/activate
source ~/.config/smart-robotic-arm/devices.env      # UNO_PORT=/dev/rfcomm0
sh bt_link.sh bind
python -m move stop                                 # 必须看到「Uno 已确认停车。」
~~~

## 11. 验收记录

日期：______  模块编号：`sjtu___`  操作人：______

| 编号 | 检查项 | 命令或操作 | 结果 |
| --- | --- | --- | --- |
| A | 名称/角色/波特率 | `AT+NAME?` `AT+ROLE?` `AT+UART?` | ☐ 通过 ☐ 失败 |
| B | 配对与信任 | `bluetoothctl info <MAC>` | ☐ 通过 ☐ 失败 |
| B | 绑定串口 | `sh bt_link.sh bind` + `ls -l /dev/rfcomm0` | ☐ 通过 ☐ 失败 |
| C | 10 次往返 | 第 4 节脚本 | ☐ 通过 ☐ 失败  中位 ____ ms |
| D | 拔模块烧录 | 第 5 节流程 | ☐ 通过 ☐ 失败 |
| E | 停车握手 | `python -m move stop` | ☐ 通过 ☐ 失败 |
| E | 单轮点动 M1–M4 | `python -m move mN --ms 200` | ☐ 通过 ☐ 失败 |
| E | 四轮前进 | `python -m move go --ms 250` | ☐ 通过 ☐ 失败 |
| E | 机械臂到格 | 在已确认可达的工位运行 `python -m arm --cell 4 --config "$CFG/arm.json"` | ☐ 通过 ☐ 失败 |
| E | 视觉闭环 | `python -m vision.align ... --cell 4` | ☐ 通过 ☐ 失败 |
| E | 共用连接 | `MHS.open_uno()` 连续动作 | ☐ 通过 ☐ 失败 |
| F | 断链后到期停车 | 架空测试，到达命令时长时确认车轮停止 | ☐ 通过 ☐ 失败 |
| F | 重连恢复 | `release` + `bind` + `stop` | ☐ 通过 ☐ 失败 |

全部必检项通过后，才进入口令到格心的完整演示。
