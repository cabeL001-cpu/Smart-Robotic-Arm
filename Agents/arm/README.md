# 两自由度机械臂

两台 MG995 和四轮共用一块 Uno。树莓派将目标格心换成肩、肘两个相对关节角，再按舵机标定换算为微秒脉宽。Uno 回复 `OK ARM` 表示已设置两路输出，格心到位由视觉测量。

## 接线和固件

基座信号默认接扩展板 **D13**，肘部接 **D7**。这是所选的两个舵机接口，按实物接线可在 `move/arm_control.ino` 修改。舵机使用合适的独立稳压电源并与 Uno 共地。安装 Arduino **Servo** 库后编译并上传整个 `move` 草图，同一份固件保留原有轮控指令。每次打开 USB 可能复位 Uno，连续操作时复用一个连接。

以下命令均从 `Agents` 目录执行。前两条可离线验证。

~~~bash
python3 -m unittest arm.test_arm
python3 -m arm --cell 4 --dry-run
arduino-cli lib install Servo
arduino-cli compile --fqbn arduino:avr:uno move
python3 -m pip install -r move/requirements.txt
python3 -m arm --joints 0 0 --port /dev/ttyACM0
python3 -m arm --cell 4 --port /dev/ttyACM0
~~~

## 几何和舵机标定

按项目图纸使用 152 mm、212 mm 两段臂长。框架内棋盘左下角为 (50, 50) mm，基座为 (−100, 150) mm。肩角从框架 +X 轴起算，肘角在两杆伸直时为零，均以逆时针为正。肩限位 ±85°、肘限位 ±135°。格心按左下 0 至右上 8 排列。逆解在可行构型中优先靠近给出的上次关节角。`--cell` 适用于基座与棋盘按图纸对齐的工位。

零位 1500 µs、每度约 7.407 µs 和 500–2500 µs 是 270° 舵机的初始配置，需按实物校正。先在脱开舵盘或有摆动余量时核对中位，逐关节点动小角度，测量方向和比例。将结果存入 JSON，以 `--config arm-calibration.json` 载入。`us_per_degree` 的符号控制旋向，`geometry` 可覆盖上述几何参数。

~~~json
{
  "shoulder": {"zero_us": 1500, "us_per_degree": 7.4074, "min_us": 500, "max_us": 2500},
  "elbow": {"zero_us": 1500, "us_per_degree": 7.4074, "min_us": 500, "max_us": 2500}
}
~~~

## 组合与独立调用

~~~python
import MHS
from arm import ArmConfig

cfg = ArmConfig.load("arm-calibration.json")
with MHS.open_uno("/dev/ttyACM0") as link:
    MHS.move("stop", link=link)
    q = MHS.arm("4", config=cfg, link=link)
    MHS.arm("5", config=cfg, link=link, previous=q)
~~~

`arm.kinematics.inverse` 和 `forward` 可用毫米坐标独立测试。`arm.set_joints` 也可直接接收规划出的关节角。串口格式为 `ARM 肩脉宽 肘脉宽` 加换行，只有合法的整对参数才更新输出。固件上电后等首个关节指令再输出脉冲。[几何与角度约束](../image-5.png) · [Arduino Servo 实现](https://github.com/arduino-libraries/Servo/blob/master/src/avr/Servo.cpp)

车辆停在不同工位时，可用单相机在当前棋盘坐标中定位基座并闭环修正。[视觉对格操作](../vision/README.md)
