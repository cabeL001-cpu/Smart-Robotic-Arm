# 前摄墙标测距与闭环直行：接口与快速开始

本文件面向没有上下文的新同事：说明这条链路上有哪些接口、怎么跑、失败时先查什么。
主线是**前摄 AprilTag 测距闭环**，配合**陀螺航向保持**完成定距直行和定角转向。

涉及两个可直接运行的入口：

| 入口 | 做什么 | 默认参数 |
| --- | --- | --- |
| `move/tag_route.py` | 前进任意距离 → 原地转向 | 73 cm，左转 90° |
| `move/south_route_tag.py` | 南侧路线：东行 → 左转 → 北行 | 73 cm，85.5°，43.5 cm |

两者都**不用固定毫秒数**：每走一小段停车，用前摄 PnP 量到墙标的距离，累计到位即停；
航向只用 YbImu 的陀螺积分，不读磁力计/AHRS 航向。

---

## 1. 硬件与软件前提

- 树莓派 + Uno（HC-05 蓝牙串口 `/dev/rfcomm0`，9600）。
- YbImu 走 USB 串口（本车 `/dev/ttyUSB0`，115200），只取陀螺。
- 前摄固定在车头、水平朝前（本车 Logitech C170，640×480，MJPG）。
- 运动期间 Uno 独立做四向超声避障，缺测或小于 300 mm 会锁停（`ERR SONAR ...` / `ERR OBSTACLE ...`）。

软件从仓库 `Agents/` 目录运行，需要 pyserial、NumPy 和带 `cv2.aruco` 的 OpenCV
（本车用 `~/mx/bin/python`）。蓝牙与设备的完整部署见 [DEPLOY_RASPBERRY_PI.md](../DEPLOY_RASPBERRY_PI.md)。

---

## 2. 五分钟快速开始

### 2.1 绑定蓝牙并确认 Uno 在线

```bash
cd ~/Smart-Robotic-Arm/Agents
sh bt_link.sh bind sjtu013          # 本车 sjtu013 = 00:21:09:00:12:EC
export UNO_PORT=/dev/rfcomm0
python3 -m move stop                # 打印“Uno 已确认停车。”即链路正常
```

树莓派重启或 HC-05 断电后 `/dev/rfcomm0` 会消失，需要重新 bind。

### 2.2 确认 IMU 和相机

```bash
ls -l /dev/ttyUSB0 /dev/rfcomm0
ls -l /dev/v4l/by-id/               # 确认前摄实际是 video0 还是别的编号
```

本车当前编号（2026-11-24）：C170 前摄 = `video0`，C270 顶摄 = `video2`。USB 重新枚举后
编号可能变，一律以 `by-id` 为准；这两个脚本只认前摄，编号不符时用 `--camera` 显式指定。

IMU 的 CH340 偶尔会在 USB 层掉线（`dmesg` 里 `error -32`，`/dev/ttyUSB0` 消失），
重新插拔后等待几秒再跑；不要用 `/dev/ttyUSB0` 打不开就怀疑代码。

### 2.3 准备两个标定文件

| 文件 | 内容 | 说明 |
| --- | --- | --- |
| `front.json` | `image_size`、`camera_matrix`、`distortion` | 必须在**实际部署分辨率**下标定；仓库里的同名文件是空模板 |
| `field.json` | `wall_family`、`wall_tags`（ID、中心、黑框边长） | 本车墙标家族 36h11，边长 200 mm |

仓库的 `move/calibration/front.json` 是空模板（`camera_matrix: []`），直接跑会报
“没有相机内参”。本车实测标定在仓库外，例如
`/home/mooc/front-cal-97Gvip/front-centered.json`，运行时显式传 `--config`。
标定步骤见 [move/CALIBRATION.md](CALIBRATION.md) 第 3 节。

### 2.4 对准墙标并检查检测

车头正对一块竖直墙标，让黑框完整入镜、留出白边。

```bash
python3 -m vision.scan front --source 0 \
  --field move/calibration/field.json --frames 1
```

输出里有 `tags[].id / center_px / corners_px` 即识别正常；`tags: []` 说明没读到，
先调方向、距离、曝光，不要继续跑闭环。

### 2.5 试运行与实跑

```bash
# 只打印计划，不接硬件
python3 -m move.tag_route --dry-run
python3 -m move.south_route_tag --dry-run

# 前进 73 cm 后左转 90°
export UNO_PORT=/dev/rfcomm0
python3 -m move.tag_route --config /path/to/front.json

# 南侧路线（本车常用写法：动作期间关闭避障）
python3 -m move.south_route_tag --config /path/to/front.json --no-avoidance
```

跑到一半想接着走：`south_route_tag` 支持 `--skip-east`（跳过东行，直接左转+北行）
和 `--no-turn`（只走东行）。

---

## 3. 接口参考

### 3.1 `TagRanger`（只测距）

`move/tag_route.py`，只读前摄，打开后可用 `measure()` 反复测距离。

```python
from vision.camera import Mount
from vision.field import Field
from move.tag_route import TagRanger

field = Field.load("move/calibration/field.json")
mount = Mount.load("/path/to/front.json")
ranger = TagRanger(0, mount, side_mm=200., family=field.wall_family)
try:
    fix = ranger.measure()          # 失败返回 None
    print(fix.tag_id, fix.z_mm, fix.rms_px)
finally:
    ranger.close()
```

构造参数：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `source` | 必填 | 相机编号、`/dev/video*` 路径 |
| `mount` | 必填 | 含 `image_size`、`camera_matrix`、`distortion` |
| `tag_id` | `None` | 指定墙标 ID；`None` 时锁定第一块读到的同家族标签 |
| `side_mm` | `200.` | 墙标黑框边长，**必须与实物一致**，否则距离按比例偏 |
| `family` | `"36h11"` | 标签家族 |

`measure(samples=5, discard=2, max_rms_px=2.5)`：先丢 2 帧旧画面，再连采 5 帧，
丢弃重投影 RMS 超过 2.5 px 的解，取各量的中位数；有效帧不足一半返回 `None`。
返回 `RangeFix(tag_id, z_mm, x_mm, y_mm, rms_px)`：`z_mm` 是镜头到标签平面的距离，
正对墙标时可当作前进距离；`x_mm/y_mm` 是标签平面内的偏移，用于判断是否歪着看。

注意：构造时会把相机分辨率设成 `mount.image_size`，读到不一致的帧会直接报错
（`输入分辨率 ... 与标定 ... 不一致`），先查 `v4l2-ctl -d <dev> --list-formats-ext`。

### 3.2 `drive_by_tag`（定距直行，航向目标固定 0°）

```python
from move.tag_route import drive_by_tag
travelled_m, final_z_mm = drive_by_tag(link, imu, ranger, distance_m=.73)
```

每段点动后停车，用陀螺把航向拉回 0°，再测一次距离；累计位移走满即停。

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `distance_m` | 必填 | 目标前进距离，米 |
| `tolerance_m` | `.015` | 允许残差 |
| `speed_mps` | `.15` | 起步速度估计，按实测自动更新 |
| `chunk_ms` | `(100, 400)` | 单段毫秒数范围 |
| `deadband_deg` | `1.5` | 航向死区 |
| `ms_per_deg` | `45.` | 短脉冲毫秒每度 |
| `min_z_mm` | `350.` | 与墙标的最小安全距离 |
| `stall_chunks` | `3` | 连续几段位移 <3 mm 判阻挡 |
| `lost_frames` | `3` | 连续几次测距失败停车 |
| `timeout_s` | `90.` | 单段总超时 |

返回 `(实测位移 m, 停车时到墙标的距离 mm)`。航向目标写死为 0°，所以只能用于
“从起步方向直走”；转向后的第二段用 3.3 的 `drive_leg`。

### 3.3 `drive_leg`（定距直行，可指定航向目标）

`move/south_route_tag.py`，参数与 `drive_by_tag` 相同，额外多一个
`target_deg`（默认 `0.`）。左转 85.5° 后的北行就传 `target_deg=85.5`，
让陀螺把航向保持在转完后的方向。

### 3.4 命令行：`move.tag_route`

```bash
python3 -m move.tag_route --help
python3 -m move.tag_route --distance-cm 73 --turn-deg 90 \
  --config /path/to/front.json --camera 0
```

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `--distance-cm` | `73` | 前进距离 |
| `--turn-deg` | `90` | 左转为正；到点后原地转向 |
| `--camera` | `0` | 前摄设备 |
| `--config` | `move/calibration/front.json` | 前摄内参（本车需显式指定实测文件） |
| `--field` | `move/calibration/field.json` | 场地与墙标尺寸 |
| `--tag` | 自动 | 指定墙标 ID |
| `--side-mm` | 取场地文件 | 墙标黑框边长 |
| `--tolerance-cm` | `1.5` | 位移容差 |
| `--no-turn` | 关 | 只走直行 |
| `--imu-port` | `/dev/ttyUSB0` | IMU 串口 |
| `--port` | `UNO_PORT` 或 `/dev/rfcomm0` | Uno 串口 |
| `--calibration-s` | `2.` | IMU 静止标定 |
| `--deadband-deg` / `--ms-per-deg` / `--yaw-scale` | `1.5` / `45.` / `1.` | 航向保持参数 |
| `--no-avoidance` | 关 | 动作期间关闭四向避障，结束时恢复 |
| `--dry-run` | 关 | 只打印计划 |

### 3.5 命令行：`move.south_route_tag`

```bash
python3 -m move.south_route_tag --config /path/to/front.json --no-avoidance
python3 -m move.south_route_tag --skip-east --config /path/to/front.json   # 接着左转+北行
python3 -m move.south_route_tag --no-turn  --config /path/to/front.json   # 只走东行
```

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `--east-cm` | `73` | 东行距离 |
| `--north-cm` | `43.5` | 左转后北行距离 |
| `--turn-deg` | `85.5` | 左转角度 |
| `--skip-east` | 关 | 跳过东行，从当前位置直接转向+北行 |
| `--no-turn` | 关 | 只走东行 |
| `--chunk-ms` | `400` | 单段点动上限（下限固定 100） |
| `--speed` | `.15` | 起步速度估计 |
| `--min-z-cm` | `35` | 与墙标最小距离 |
| `--tolerance-cm` | `1.5` | 位移容差 |
| 其余 | 同 3.4 | 相机、场地、IMU、航向参数、避障 |

未指定 `--tag` 时，`--skip-east` 或转向后的那一段会**自动重新锁定**视野里的第一块墙标
（转向后前方可能换了一面墙）。

---

## 4. 原理

### 4.1 测距：单码 PnP，取 Z

检测器给出黑框四角的像素顺序（左上起顺时针）。`TagRanger` 用与之一致的
Y 向上物点：

```text
[-h, +h, 0] [ +h, +h, 0] [ +h, -h, 0] [-h, -h, 0]      h = side_mm / 2
```

再调用 `cv2.solvePnP(..., flags=cv2.SOLVEPNP_IPPE_SQUARE)`。物点顺序弄反会得到
镜像解（曾出现 z≈22 mm、方位 −179° 的假值），所以顺序不能随便改。
解出的平移向量 `t` 第二分量（Z）就是镜头到标签平面的距离；正对时即前进方向距离。

- 内参来自 `front.json`，帧先按标定分辨率使用；分辨率不符直接报错。
- 每帧算重投影 RMS，超过 2.5 px 的帧丢弃；`measure` 取 5 帧中位数。
- 距离精度依赖 `side_mm`：按 200 mm 假设，焦距标定的 2–4% 不确定度会等比例带进距离。

### 4.2 闭环：位移 = 起步距离 − 当前距离

```text
剩余 = 目标 − 已累计
单段 = clamp(剩余 / 速度 × 1000, 100, 400) ms
发 GO → 停车 → 陀螺修航向 → 再测一次 → 累计 += (上次距离 − 本次距离)
```

因为每段都停车再拍，画面不会拖影；累计的是**测出来的距离差**，不是里程计。
速度估计按 `累计位移 / 已发毫秒` 在线更新，并夹在 0.06–0.30 m/s。

### 4.3 航向：只用陀螺积分

`open_imu()` 先静止采样 2 s 做零偏，之后 `ImuFeed.snapshot_gyro()` 对陀螺竖直分量做
梯形积分。实车磁力计受电机、电池、车架干扰（手推 90° 四个方向读数 59°/75°/142°/83°，
静止还会回弹），所以**刻意不用** AHRS 航向。

`correct_heading`：偏差 = 目标 − 当前；超过死区就发
`rotate-left/right`，脉宽 `|偏差| × ms_per_deg`（默认 45 ms/°，夹在 65–300 ms），
最多 3 次。`--yaw-scale` 用于修正陀螺比例（= 90 ÷ 手推 90° 时的读数）。

### 4.4 转向：`turn_to`

第一次按 90°≈4000 ms 粗转，之后用短脉冲增益（默认 45 ms/°）修正，死区 1°，
最多 8 次；返回最终偏差。短脉冲有起步加速，所以不能用 44.4 ms/° 直接算。

### 4.5 安全策略

- 起步前检查：`起步距离 − 目标距离 < 最小距离` 直接拒绝，避免撞墙。
- 每段测距失败连续 `lost_frames` 次 → 停车报错。
- 连续 `stall_chunks` 段位移 <3 mm → 判定被阻挡或方向错误，停车报错。
- 单段超时 90 s → 停车报错。
- 测距低于最小距离 → 立即停车。
- 底盘动作期间 Uno 的超声避障独立生效，触发时抛出 `MotionBlocked`。
- `--no-avoidance` 只是临时关闭固件避障（`AVOID OFF`），`finally` 中一定恢复；
  用于超声探头误报或后方净空不足的场合，**关闭后碰撞保护由操作者负责**。

---

## 5. 本车实测记录（2026-11-23/24）

| 动作 | 目标 | 实测 | 备注 |
| --- | --- | --- | --- |
| 前进 + 左转 | 73 cm、90° | 72.5 cm，最终航向偏差 0.0° | 墙标 tag 3，起步 1.83 m |
| 前进 | 40 cm | 40.6 cm（1119→713 mm） | tag 1 |
| 东行 | 73 cm | 72.3 cm（1838→1116 mm） | 12 段 400 ms，航向 ≤0.7° |
| 左转 | 85.5° | 最终偏差 0.0° | 3800 ms + 223 ms 修正 |
| 北行 | 46 cm | 46.5 cm（1095→631 mm） | 航向 ≤0.4° |

北行默认距离已改为 43.5 cm，尚未实车复测。

---

## 6. 故障排查

| 现象 | 原因与处理 |
| --- | --- |
| `没有相机内参` | `--config` 指到了空模板；换成实测标定文件 |
| `输入分辨率 (640, 480) 与标定 (1280, 720) 不一致` | 相机编号/标定文件不匹配；用 `by-id` 确认前摄，或换对应分辨率的标定 |
| `起步时前摄读不到墙标` | 墙标不在视野、太远/太小、模糊或反光；先跑 `vision.scan front` |
| `当前距离 ... 不足以安全前进` | 起步太近，退后或减小目标距离（`--min-z-cm` 可调但别小于实际刹车距离） |
| `直行中连续读不到墙标` | 行驶中墙标出画/被挡；检查相机固定、路线是否正对墙标 |
| `连续点动测距无明显变化` | 车轮没转、被挡、方向反了；先点动验证 `go` |
| `底盘因避障保持停车` | 超声 <300 mm 或缺测；检查探头与净空，确认安全后再考虑 `--no-avoidance` |
| `could not open port /dev/ttyUSB0` | IMU 掉线；插拔后等枚举，`dmesg` 确认 `ch12-uart` 挂上 |
| IMU 静止标定超时 | 车在动或数据流断；停稳后重试 |
| `device reports readiness to read but returned no data` | 相机首次打开瞬时失败；重跑一次，连续出现再查 USB 供电/线材 |

---

## 7. 已知限制

- 一次测量只用**一块**墙标，距离是沿标签法向的；斜视超过约 30° 时 z 会偏小。
- 墙标边长必须是实测值；换打印尺寸后所有距离等比例变化。
- 陀螺方案只保“相对起步的航向”，不提供绝对方向；IMU 掉线/重启后需要重新标定。
- 该链路不做横向纠偏：车体左右偏移只会被“斜着看到标签”间接反映，长距离路线仍需
  车体本身跑直、必要时用 `vision.place` 的车体闭环或前摄定位修正。
- `--no-avoidance` 下没有碰撞保护；只在确认净空时使用。