# YbImu 九轴平面定位

IMU 用独立的 USB 串口接树莓派，默认协议为 115200、8N1。底盘和机械臂共用的 Uno 由车载电源供电，经另一位同学提供的蓝牙链路通信。用 `python3 -m serial.tools.list_ports -v` 核对 IMU 的 USB 节点，并单独记录蓝牙端点。下列命令均从 `Agents` 运行。

## 实时运行与原始数据记录

将车停在已知原点，车头朝场地 +X，开机后保持静止至少两秒。若从其他已知位姿启动，用 `--origin X米 Y米 朝向度` 给出初值。传感器 +X 朝车头时安装偏角为零，其他安装方向用 `--mount-yaw` 指定传感器 +X 相对车头的逆时针角度。

~~~bash
python3 -m pip install -r move/requirements.txt
python3 -m move.imu --port /dev/ttyUSB0 --record run-raw.csv
python3 -m move.imu --replay run-raw.csv
python3 -m unittest move.imu.test_imu
~~~

每个新样本输出一行 JSON，包括单调时间戳、场地位置（米）、航向（弧度）、速度（米/秒）和本次是否接受视觉校正。台架验证时可加 `--stationary` 连续施加零速约束。在线程序也可在确认底盘静止时调用 `tracker.update(sample, stationary=True)`。

## 估计原理与视觉校正

解析厂商的 `7E 23` 帧头、总长度、功能字和模 256 累加校验。原始帧 `0x04` 提供三轴加速度与角速度，姿态来自 `0x16` 四元数或 `0x26` 欧拉角。静止段估计传感器坐标中的加速度偏置，AHRS 四元数将比力旋到导航系，随后在场地水平面进行梯形积分。采样缺口大于 0.3 秒时冻结位置并清零速度，等待后续样本和绝对观测。

IMU 给出短时间相对位移，零偏会随积分产生位置漂移。相机看到场地固定标记后，可用同一树莓派上的曝光时间校正历史位姿，再保留曝光后发生的运动。程序接受最近一秒内的有序观测。无标记且搬动期间传感器未记录运动时，绝对位置需等下一次有效视觉观测才能恢复。

~~~bash
python3 -m vision.robot --source 0 --config vehicle.json | python3 -m move.imu --port /dev/ttyUSB0 --vision-stdin
python3 -m move.imu --replay run-raw.csv --fixes poses.jsonl
~~~

两个实时进程使用同一台树莓派的单调时钟。离线回放要使用同次录制、同一时基的视觉 JSON 行。输入模块也可直接调用 `Tracker.correct(t, x_m, y_m, yaw_rad)`，用返回值判断观测是否被采用。[视觉位姿与相机标定](../../vision/README.md)

## 资料与可重复验证

串口字段及单位依据提供的 YbImuLib 树莓派驱动。厂商上位机可配置九轴融合及回传频率，并可完成硬件水平校准和磁力计校准。这里每次启动进行软件零偏与航向对齐。

记录文件保存 `time_s,ax_g,ay_g,az_g,gx_rad_s,gy_rad_s,gz_rad_s,qw,qx,qy,qz`。资料目录内的 `imu_trajectory_*.csv` 保存的是已经计算出的轨迹，不能据此重算原始加速度。独立测试覆盖协议拆包、坏校验、两类姿态帧、倾斜静止、加速与匀速、停车、断采和延迟视觉重定位。
