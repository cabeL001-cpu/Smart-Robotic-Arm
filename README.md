# Smart-Robotic-Arm · 银立方实验机器人

树莓派通过 HC-05 蓝牙控制 Uno。Uno 驱动麦克纳姆四轮底盘、两台 MG995 舵机并轮询四个 SR09 探头。小车用 IMU 从启动零点出发，在目标附近整圈扫描前摄墙标，测量车心并微调，遇障后锁停。随车顶摄用于九宫格、机械臂末端和车体定位。

| 我要做什么 | 从哪里开始 |
| --- | --- |
| 首次装机、接线、配置树莓派 | [树莓派部署指南](Agents/DEPLOY_RASPBERRY_PI.md) |
| 用 Arduino CLI 给 Uno 烧录 | [USB 烧录与校验](Agents/DEPLOY_RASPBERRY_PI.md#4-将合并固件烧录到-uno) |
| 通过蓝牙连接小车、检查停车回执 | [HC-05 配对与验收](Agents/BLUETOOTH_WORKFLOW.md) |
| 调用导航、底盘、机械臂、视觉、IMU 或语音 | [接口与参数手册](Agents/API.md) · [模块入口](Agents/README.md) |
| 配置 A/B/C/D 或任意坐标，校准前摄、IMU、轮速和避障 | [小车标定与运行](Agents/move/CALIBRATION.md) |
| 标定两轴限位、顶摄和指尖，完成九宫格任务 | [机械臂标定与到位](Agents/arm/CALIBRATION.md) |

**无硬件预览。** 从仓库根目录运行下面两行，终端应打印 `GO 250`。此命令只需 Python 标准库。

```sh
cd Agents
python -m move go --ms 250 --dry-run
```
