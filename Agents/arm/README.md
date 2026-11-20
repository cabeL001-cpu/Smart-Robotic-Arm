# 两自由度机械臂

两台 MG995 和四轮共用一块 Uno。肩部信号接 **D13**，肘部信号接 **D10**，舵机稳压电源与 Uno 共地。实车控制数据经扩展板上的 HC-05 蓝牙串口 `/dev/rfcomm0` 传输，不占 USB。`OK ARM` 表示 Uno 已设置两路输出，格心到位由相机测量。

**从旧接线改到 D10**

1. 支撑机械臂，断开车载电源、舵机电源并拔掉 Uno 的 USB 线。把肘部 MG995 的**信号线**从 D7 移到 Uno 数字排针标有 **10** 或 **~10** 的针位。舵机正线接外部稳压电源正极，地线接负极，负极与 Uno `GND` 共地。
2. 扩展板的绿色双针 `VOUT` 也受 D10 控制。断电时拔掉接在 `VOUT` 上的负载，使用肘部舵机期间让该端子保持空载。[扩展板端子图](../image-8.png)
3. 在烧录用的工程中按下文从实测 `arm.json` 导出限位，再通过 USB 上传完整的 `move` 草图（烧录前拔下 HC-05）。只移动信号线时可以沿用原有实测标定。上传后拔掉 USB，把 HC-05 插回蓝牙座并恢复车载供电与 `/dev/rfcomm0` 绑定。将肩角设为当前实测角，让肘角在已标定行程内小幅变化，先预览再点动。[限位导出与点动命令](#预览生成固件限位验收)

## 实车行程与关节坐标

从各舵机的机械最大位置向回只允许转 **90°**。最大位置是物理止挡。肩部照片中连杆位于止挡时向右上方倾斜，照片角度受透视影响。肩角以框架 +X 为零，肘角以两杆伸直为零，俯视逆时针为正。肩部照片只提供肩关节的姿态参考，肘部需独立测量。

肩部信号接扩展板 **D13 的 S**，肘部信号接 Uno 数字排针 **D10**（丝印 `10` 或 `~10`；扩展板的绿色 `VOUT` 也由 D10 控制，使用肘部舵机期间保持该端子空载）。按实物接线可在 `move/arm_control.ino` 修改。舵机使用合适的独立稳压电源并与 Uno 共地。安装 Arduino **Servo** 库后编译并上传整个 `move` 草图，同一份固件保留原有轮控指令。树莓派经 HC-05 蓝牙串口 `/dev/rfcomm0` 下发命令，打开链路不会复位 Uno；走 USB 台架调试时每次打开可能复位，连续操作时复用一个连接。

每个关节的 `angle_at_max_deg` 填**轴在最大位置时连杆的几何关节角**。若从止挡退回时几何角减小，`retreat_sign` 填 `-1`，行程是 `[angle_at_max_deg − 90°, angle_at_max_deg]`。若因安装方向相反而增大，填 `1`。软件默认两端各留 `3°`，再与图纸肩 ±85°、肘 ±135°、实测脉宽范围取交集。逆解逐个检查两个肘部构型，只有可行的构型才会下发。均不可达时返回错误。改变停靠位会改变可达格子。

## 如何得到标定值

1. 支撑臂并断开舵机负载电源。在已观察到的止挡位置，用与转轴平面平行的量角器或正上方照片量连杆角度。肩部用大臂相对框架 +X，肘部用小臂相对大臂。若用尺量，记录肩轴指向肘轴的平面投影 Δx、Δy，肩角为 `atan2(Δy, Δx) × 180/π`。肘角取小臂方向减大臂方向的有符号角。轻轻退开一小段记录角度变化方向。停止继续用力推止挡，也不要通过持续通电堵转寻找端点。如果退开后仍卡滞，先脱开连杆检查机构。
2. 用已有的已知安全脉宽记录，或在卸载、标记好舵盘安装方向的台架上用能显示微秒的舵机测试器，在**行程内部**得到两组脉宽与几何角 `(p1, q1)`、`(p2, q2)`。每轴单独算 `us_per_degree = (p2 − p1)/(q2 − q1)`，`zero_us = p1 − q1 × us_per_degree`。允许负斜率。零位脉宽是线性换算截距，零角可能位于当前行程以外。
3. 在树莓派的 `$CFG/arm.json`（或编译机上的同名文件）填写实测数据。下例中的 `null` 表示待量，电气参数来自旧配置的名义值，须用前一步结果核对或替换。`min_us`、`max_us` 可进一步收窄。旧 JSON 可加载，但缺少机械端点时会在发指令前报告缺项。

~~~json
{
  "shoulder": {"angle_at_max_deg": null, "retreat_sign": -1, "margin_deg": 3,
               "zero_us": 1500, "us_per_degree": 7.4074, "min_us": 500, "max_us": 2500},
  "elbow":    {"angle_at_max_deg": null, "retreat_sign": -1, "margin_deg": 3,
               "zero_us": 1500, "us_per_degree": 7.4074, "min_us": 500, "max_us": 2500}
}
~~~

## 预览、生成固件限位、验收

均从 `Agents` 运行。在实际负责烧录的那份源码中执行导出。导出只写头文件，随后编译上传整个 `move` 草图，Uno 才会采用新限位。关节重新装配或标定改变后重复导出和烧录。仓库附带的占位头文件使 Uno 回复 `ERR ARM CALIBRATION`，四轮和 SR09 可独立使用。

~~~bash
python -m unittest arm.test_arm
python -m arm --config "$CFG/arm.json" --export-limits move/arm_limits.h
arduino-cli compile --fqbn arduino:avr:uno move
python -m arm --config "$CFG/arm.json" --cell 4 --dry-run
~~~

先选刚测过、两轴都在行程内的姿态。从该姿态附近小幅点动，确认两根连杆的方向与角度，再尝试可达格心。底盘保持静止。

~~~bash
read -r -p "实测肩角和肘角（度，以空格分隔） " Q1 Q2
python -m arm --config "$CFG/arm.json" --joints "$Q1" "$Q2" --dry-run
python -m arm --config "$CFG/arm.json" --joints "$Q1" "$Q2" --port "$UNO_PORT"
~~~

两段名义臂长为 152 mm、212 mm。`--cell` 使用图纸对齐工位，左下为格 0，右上为格 8。`vision.align` 在实测基座姿态下使用同一套限位规划。Uno 收到 `ARM 肩脉宽 肘脉宽` 后，先检查两路，任一越界都回复 `ERR ARM` 并保持原输出。首次挂接时两条合法脉宽同时就绪，避免发出默认中位脉冲。[几何图](../image-5.png) · [视觉闭环](../vision/README.md)

## 组合调用

~~~python
import os
from pathlib import Path
import MHS
from arm import ArmConfig

cfg = ArmConfig.load(Path.home() / ".config/smart-robotic-arm/arm.json")
with MHS.open_uno(os.environ.get("UNO_PORT", "/dev/rfcomm0")) as link:
    MHS.move("stop", link=link)
    q = MHS.arm("4", config=cfg, link=link)
~~~

`arm.kinematics.inverse` 可用于独立的图纸几何计算。`ArmConfig.solve`、`MHS.arm`、点动和视觉闭环都会检查实车标定。
