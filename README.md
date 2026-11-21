# Smart-Robotic-Arm · 银立方实验机器人

树莓派通过 HC-05 蓝牙控制 Uno。Uno 驱动麦克纳姆四轮底盘、两台 MG995 舵机和四个 SR09 探头。随车下视相机用于九宫格和车体定位，前视相机可独立读取围挡标签。Python 模块、Uno 草图和接线说明都在 `Agents/`。

| 我要做什么 | 从哪里开始 |
| --- | --- |
| 首次装机、接线、配置树莓派 | [树莓派部署指南](Agents/DEPLOY_RASPBERRY_PI.md) |
| 用 Arduino CLI 给 Uno 烧录 | [USB 烧录与校验](Agents/DEPLOY_RASPBERRY_PI.md#4-将合并固件烧录到-uno) |
| 通过蓝牙连接小车、检查停车回执 | [HC-05 配对与验收](Agents/BLUETOOTH_WORKFLOW.md) |
| 调用底盘、机械臂、视觉、IMU 或语音 | [系统接口与各模块说明](Agents/README.md) |

**无硬件预览。** 从仓库根目录运行下面两行，终端应打印 `GO 250`。此命令只需 Python 标准库。

```sh
cd Agents
python -m move go --ms 250 --dry-run
```
