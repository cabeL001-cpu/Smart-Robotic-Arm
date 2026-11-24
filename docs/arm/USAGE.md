# 机械臂函数使用与原理

本文说明 `arm` 模块（`python -m arm`、`MHS.arm`）怎么用、内部怎么算，以及当前这辆车为什么还不能直接用它做开环入格。怎么得到标定数值见[机械臂标定](CALIBRATION.md)；视觉闭环入格见[视觉模块](../vision/README.md)。

## 1. 两条可用路径

| 路径 | 入口 | 需要什么 | 现状 |
| --- | --- | --- | --- |
| 开环规划 | `python -m arm --cell N`、`MHS.arm(N, config=cfg)` | 完整的 `arm.json`：几何尺寸、基座位姿、两轴脉宽—角度模型 | 树莓派上的 `arm.json` 仍是 11-20 占位，缺 `angle_at_max_deg`，规划阶段即报错，不发舵机指令 |
| 视觉闭环 | `python -m vision.servo --cells N`、`python -m vision.place --cells N` | 顶摄相机、实测预置脉宽 `CELL_SEEDS` | 已验收：0/3/6/7/8 格误差 0.73–2.08 mm；1/2/4/5 需配合车体微调 |

开环规划算的是“图纸工位”下的格心；视觉闭环直接用相机测指尖与棋盘，不依赖角度模型。**当前要把末端放进格心，用第二条。**

## 2. 原理

### 2.1 硬件与串口协议

- 两自由度平面臂：肩接 Uno **D13**，肘接 **D10**；两台 MG995 由独立稳压电源供电，与 Uno 共地。
- 树莓派经 HC-05 蓝牙串口 `/dev/rfcomm0` 下发命令，与四轮共用一块 Uno。
- 命令：`ARM <肩µs> <肘µs>\n` → `OK ARM`；越界回 `ERR ARM`；固件限位未配置回 `ERR ARM CALIBRATION`；`ARM OFF\n` → `OK ARM OFF` 解除保持力矩。
- `OK ARM` 只表示 Uno 已设置两路脉宽，**是否到位必须用相机确认**。

### 2.2 平面二连杆运动学（`arm/kinematics.py`）

格心按图纸坐标给出：棋盘左下在 `(50, 50)` mm，格距 `200/3` mm，格号从左下起逐行递增。

正解：

```
x = base_x + L1·cos(θ1 + base_yaw) + L2·cos(θ1 + θ2 + base_yaw)
y = base_y + L1·sin(θ1 + base_yaw) + L2·sin(θ1 + θ2 + base_yaw)
```

逆解先由余弦定理求肘部弯曲量 `bend = acos((d² − L1² − L2²) / (2·L1·L2))`，再对 `±bend` 两种构型分别求肩角，保留同时满足几何角限和实测脉宽限位的解；两解都可行时取距上次关节角最近的一组（`previous`）。没有可行解时报“目标超出机械臂工作半径”或“目标超出关节角度范围”。

### 2.3 脉宽—角度模型（`ServoCalibration`）

每个关节由一组参数描述：

| 字段 | 含义 |
| --- | --- |
| `zero_us` | 线性截距，`脉宽 = zero_us + us_per_degree × 角度` |
| `us_per_degree` | 有符号斜率，允许为负 |
| `angle_at_max_deg` | 轴在机械最大位置时连杆的几何角 |
| `retreat_sign` | 从最大位置退回时几何角变化的符号，±1 |
| `margin_deg` | 两端各留的余量 |
| `min_us` / `max_us` | 电气可用边界（代码要求 500–2500） |

软件只允许从止挡退回 **90°**：

```
低角 = angle_at_max_deg + retreat_sign × (90 − margin_deg)
高角 = angle_at_max_deg + retreat_sign × margin_deg
```

再把这段角度换算成脉宽，与 `min_us`/`max_us` 和 `Geometry` 的设计角限取交集，向内取整后作为该关节的可执行区间。目标角落在区间外就拒绝下发。

### 2.4 固件限位

`python -m arm --export-limits move/arm_limits.h` 用同一份 `arm.json` 生成固件硬限位。Uno 对每条 `ARM` 先查两路脉宽，任一越界都不动作。当前车上固件为肩 50–1080 µs、肘 50–2700 µs（11-23 扫端点后放宽），实际流畅窗口见第 4 节。

### 2.5 视觉闭环（当前实际使用）

- 顶摄用棋盘角标拟合单应，把末端标签（解不出时退到黑框四边形）的中心换算成棋盘毫米。
- 每格先摆到实测预置脉宽 `CELL_SEEDS`，再分别给肩、肘加 60 µs 探测，测出 2×2 雅可比（毫米/微秒），按 `J·Δp = 目标 − 指尖` 牛顿迭代，单轴限幅、脉宽钳位。
- 九宫格相对车体位置变了（车没停回标定位姿）时，预置位失效，改用 `--closed`：从当前脉宽直接开始，每轴自适应探测（60→240 µs，测完回基准位）、多帧中位数观测、阻尼最小二乘、末段从目标外侧 3 mm 单向接近，到位后连续三次复核。
- 连续多帧看不到末端标签就停在该处，不盲动；到位后保持力矩，不自动 `ARM OFF`。
- 车体够不到的格由 `vision.drive` 接管：臂固定在 `(360, 1570)`，用前进/后退/左右平移的车体雅可比微调。

## 3. 使用

### 3.1 前置检查

```bash
ls /dev/ttyACM* 2>/dev/null     # 有输出说明 Uno 还连着 USB：先拔掉，否则舵机供电不足
ls -l /dev/rfcomm0              # 蓝牙串口节点
stty -F /dev/rfcomm0 raw -echo
```

`export UNO_PORT=/dev/rfcomm0`（或使用 `~/.config/smart-robotic-arm/devices.env`）。以下命令都在 `~/Smart-Robotic-Arm/src` 下运行。

### 3.2 预览、点动、执行

```bash
CFG=~/.config/smart-robotic-arm/arm.json

# 只算角度和脉宽，不发指令
python3 -m arm --config "$CFG" --cell 4 --dry-run

# 已知角度的点动：先预览，再执行
python3 -m arm --config "$CFG" --joints 60 90 --dry-run
python3 -m arm --config "$CFG" --joints 60 90 --port "$UNO_PORT"

# 按格心执行
python3 -m arm --config "$CFG" --cell 4 --port "$UNO_PORT"

# 从实测标定导出固件限位（不打开串口）
python3 -m arm --config "$CFG" --export-limits move/arm_limits.h
```

`--cell` 使用图纸对齐工位；改停靠位会改变可达格。底盘保持静止，臂下方清空，首次只跑 `--dry-run`。

### 3.3 Python 调用

```python
import os
from pathlib import Path
import MHS
from arm import ArmConfig

cfg = ArmConfig.load(Path.home() / ".config/smart-robotic-arm/arm.json")
with MHS.open_uno(os.environ.get("UNO_PORT", "/dev/rfcomm0")) as link:
    MHS.move("stop", link=link)
    angles = MHS.arm("4", config=cfg, link=link)   # 返回 (肩角, 肘角)
```

`MHS.dispatch((格号, "arrive"), arm_config=cfg)` 是语音任务的分派入口；没有 `arm_config` 会直接报“对格任务需要 0–8 的格号与实测 arm_config”。

### 3.4 入格（视觉闭环）

```bash
export UNO_PORT=/dev/rfcomm0
python3 -m vision.servo --cells 7 --targets board_targets.json --raw
# 新仓库（已 git pull）还可自动分流不可达格：
python3 -m vision.place --cells 1 4 --targets board_targets.json --raw
```

上面的 `vision.servo` 会先摆 `CELL_SEEDS` 预置位。如果**车没停回 11-23 标定位姿、九宫格相对车体挪了位置**，预置位本身就不对了，改用全闭环模式：给当前臂脉宽当起点，之后每步都由当前画面决定，不摆任何预置位。

```bash
python3 -m vision.servo --closed --start 500 1200 --cells 3 --raw --targets board_targets.json
```

`--start` 是当前肩、肘脉宽（固件读不回位置，必须人工给）；起点看不到末端方块时一次也不动。单格动作次数比预置位模式多，`--tolerance` 缺省 4.0 mm（实测有效重复精度）、`--max-steps` 缺省 18（跨格长距离要放到 36–40）。详见[视觉模块](../vision/README.md)。

如果车停回的位置与 11-23 标定 0/3/6/7/8 时不同、可达点明显变差，先微调回标定位姿再入格：

```bash
python3 -m vision.reposition --raw --dry-run   # 只量当前偏差，不动车
python3 -m vision.reposition --raw             # 用车底四轮微调到标定位姿
```

## 4. 配置与现有实测数据

完整配置示例（数值未标定处为 `null`，必须先替换；`geometry` 单位毫米、度）：

```json
{
  "geometry": {"upper_mm": 152, "forearm_mm": 212, "base_x_mm": -100,
               "base_y_mm": 150, "base_yaw_deg": 0},
  "shoulder": {"angle_at_max_deg": 100, "retreat_sign": -1, "margin_deg": 0,
               "zero_us": -120, "us_per_degree": 12, "min_us": 500, "max_us": 1080},
  "elbow": {"angle_at_max_deg": null, "retreat_sign": -1, "margin_deg": 0,
            "zero_us": 1450, "us_per_degree": -10, "min_us": 500, "max_us": 2500}
}
```

文档记录的实测值：

| 项目 | 数值 | 来源 |
| --- | --- | --- |
| 肩脉宽—角度 | `zero_us=-120`、`us_per_degree=12`；500→约 50°、900→85°、1200→110°、1400 顶止挡 | 11-22 量角器扫描 |
| 肩流畅窗口 | 360–640 µs（≤320 不动，700 起抖，1000 卡止挡区） | 11-23 复测 |
| 肘相对关系 | `zero_us=1450`、约 `-10 µs/°`；650 µs ≈ 1450 逆时针 90° | 11-22 标签朝向 |
| 肘端点 | 450 µs 起有动作，2550 µs 为上界 | 11-23 扫端点 |
| 固件限位 | 肩 50–1080 µs、肘 50–2700 µs | 11-23 烧录 |
| 每格预置位 | 0:(500,640) 3:(360,1162) 6:(463,1429) 7:(360,1570) 8:(360,1920) | 11-23 实测 |

**为什么现在开环跑不了**：`arm.json` 缺 `angle_at_max_deg`（校验直接报错）；肩的实测流畅窗口 360–640 µs 与配置模型/`min_us≥500` 校验不一致；肘部只有相对斜率，1450 µs 对应的绝对几何角没定，基座位姿也未标定。补齐这些后才能用 `--cell` 规划；在此之前用第 3.4 节的视觉闭环。

## 5. 错误对照

| 报错 | 原因 | 处理 |
| --- | --- | --- |
| `缺少 angle_at_max_deg，请先标定该关节的机械最大位置` | 配置仍是占位或不完整 | 按[标定文档](CALIBRATION.md)补全两轴字段 |
| `舵机标定的旋向、端点余量、斜率或脉宽范围无效` | `retreat_sign` 不是 ±1、`margin_deg` 越界、斜率 0，或 `min_us/max_us` 不在 500–2500 | 核对配置数值 |
| `机械行程与图纸角限没有交集` / `机械行程与可用脉宽没有交集` | 90° 行程、几何角限与电气限位对不上 | 收窄或平移 `angle_at_max_deg`、`min_us/max_us` |
| `目标 xx° 超出已标定脉宽区间 [a, b] µs` | 格心角度落在可执行区间外 | 换停靠位，或确认标定 |
| `目标超出机械臂工作半径` / `目标超出关节角度范围` | 格心不在图纸工位可达范围 | 检查基座位姿与几何尺寸，或改用视觉闭环 |
| `ERR ARM CALIBRATION` | 固件限位未烧录 | 导出 `arm_limits.h` 并重新烧录 |
| `ERR ARM` | 脉宽越界 | 核对两轴脉宽是否在固件窗口内 |
| `TimeoutError` / 打开串口失败 | HC-05 未绑定或链路未建立 | `sh bt_link.sh bind`，检查 `/dev/rfcomm0` |
| `对格任务需要 0–8 的格号与实测 arm_config` | `MHS.dispatch` 未传配置 | 传入 `ArmConfig.load(...)` |

## 6. 安全

- **Uno 接 USB 时不要驱动舵机**：USB 供电不足会让舵机堵转、抖动或掉电。USB 只用于烧录。
- 只允许从机械止挡退回 90°；不要通电顶住止挡找端点，卡滞时先断电、支撑并脱开连杆。
- 竖直姿态悬空时不要 `ARM OFF`，臂会落下砸到棋盘；确认下方清空再解除力矩。
- 视觉闭环看不到标签时会停在当前位。这是有意行为：不要为了“走过去”而放开车体盲动。
- 改停靠位、换舵盘或改固件限位后，可达格与预置位都要重新实测。

## 7. 快速调用接口（无上下文版）

给接手的人只需要下面这一段。前提：树莓派已登录、相机在 `/dev/video0`（顶摄）、HC-05 已配对为 `/dev/rfcomm0`、Uno **没有**接 USB。

**最短路径：把末端放进指定格（视觉闭环，不依赖 `arm.json`）**

```bash
cd ~/Smart-Robotic-Arm/src
export UNO_PORT=/dev/rfcomm0
ls /dev/ttyACM* 2>/dev/null    # 有输出就先拔 USB，否则舵机供电不足

python3 -m vision.servo --cells 4 --targets board_targets.json --raw
```

- 换格：改 `--cells`，可写多个，如 `--cells 0 3 6 7 8`。
- 车没停回 11-23 标定位姿、九宫格相对车体挪了位置时，预置位失效，改用全闭环：
  `python3 -m vision.servo --closed --start 肩µs 肘µs --cells 3 --raw --targets board_targets.json`（`--start` 给当前脉宽，固件读不回位置）。
- 1、2、4、5 格臂单独够不到；仓库已 `git pull` 时改用自动分流：
  `python3 -m vision.place --cells 1 4 --targets board_targets.json --raw`（车体微调会自动关避障、结束恢复）
- 车停的位置与 11-23 标定 0/3/6/7/8 时不同、可达点变差，先跑
  `python3 -m vision.reposition --raw` 把车微调回标定位姿，再入格；先加 `--dry-run` 只量偏差。
- 输出每格一行 JSON：`ok` 是否到位、`error_mm` 残余误差、`note` 停车原因（看不到末端标签时会停在当前位）。

**如果你确实已有完整标定的 `arm.json`（开环按格心规划）**

```bash
python3 -m arm --config ~/.config/smart-robotic-arm/arm.json --cell 4 --dry-run   # 先预览
python3 -m arm --config ~/.config/smart-robotic-arm/arm.json --cell 4             # 再执行
```

**Python 里一行调用**

```python
import MHS
from arm import ArmConfig
cfg = ArmConfig.load("/home/mooc/.config/smart-robotic-arm/arm.json")  # 必须是完整标定文件
angles = MHS.arm(4, config=cfg, port="/dev/rfcomm0")                    # 返回 (肩角, 肘角)
```

**只需记住 5 件事**

1. 先 `export UNO_PORT=/dev/rfcomm0`，Uno 接 USB 时不能驱动舵机。
2. 现在这辆车用 `vision.servo` / `vision.place` 入格；`python -m arm --cell` 需要补全 `arm.json`（见第 4 节）。
3. `--dry-run` 只算不发指令，第一次一定先跑它。
4. `OK ARM` 只代表脉宽已下发，是否到位看相机；视觉闭环输出 `ok` 和 `error_mm`。
5. 视觉闭环看不到末端标签就会停车，不会盲动；竖直臂悬空时不要 `ARM OFF`。