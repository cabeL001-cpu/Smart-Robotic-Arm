# 小车标定与坐标导航

本流程标定从**车心**到目标的地面位置。每次开机先静止建系，依靠 IMU 粗走，在目标附近原地扫一整圈墙面 AprilTag，再以停车后的新图像微调。导航全程由 Uno 独立检查前、后、左、右净空，一旦遇障或缺测就锁停。表中的场地尺寸取自任务图，实车使用时逐项替换为实测值。[接线与设备环境](../DEPLOY_RASPBERRY_PI.md) · [接口、返回值与固件协议](../API.md)

## 1. 准备设备和标定文件

- **Uno** 经 HC-05 蓝牙与树莓派通信，运行支持 `NAV BEGIN/MOVE/CHECK/END/RESET` 的完整 `move` 草图。旧固件即使手动点动正常也需要更新。保留本车由 `arm.json` 生成的两轴限位。烧录时断电拔下与 USB 共用 D0/D1 的 HC-05，完成后恢复车载供电和蓝牙。[固件上传步骤](../DEPLOY_RASPBERRY_PI.md#4-将合并固件烧录到-uno)
- **IMU** 用独立 USB 串口。**前摄**固定在车头看竖直墙标。顶摄仍向下看九宫格，约 1.5 m 的高度只属于顶摄。`devices.env` 中 `UNO_PORT`、`IMU_PORT`、`FRONT_CAM` 分别指向蓝牙、YbImu 和前摄真实设备。关闭其他占用对应设备的进程。
- 程序使用 pyserial、NumPy 和带 `cv2.aruco` 的 OpenCV。[依赖安装](../DEPLOY_RASPBERRY_PI.md#2-准备树莓派系统和代码) 现场从仓库 `Agents/` 执行下列命令，`cp -i` 在已有标定时会询问是否覆盖。

~~~bash
export CFG="$HOME/.config/smart-robotic-arm"
export NAVCFG="$CFG/navigation"
source "$CFG/devices.env"
mkdir -p "$NAVCFG"
cp -i move/calibration/nav.json move/calibration/field.json move/calibration/front.json "$NAVCFG/"
~~~

| 文件 | 要填什么 | 何时重标 |
| --- | --- | --- |
| `field.json` | 车心范围、停靠坐标、各墙标的家族、编号、中心、高度、朝向和黑框边长 | 围挡、停靠线或标签移动 |
| `front.json` | 前摄实收分辨率、内参、畸变、光心和朝向 | 更换镜头、分辨率、裁切或移动机架 |
| `nav.json` | 相对本文件的地图和前摄路径、起点模式、IMU 安装角、六向实测速率、闭环参数 | 换起点工位、改变电池、载荷、地面或轮系 |

模板中 `null` 速率、空相机参数和墙标中心高度都是待测项。不要把空值填为 0 以启动。填写后，将三份文件与场地草图、设备编号和日期一同保存。

## 2. 量场地、目标和起点

1. 在地上定一个可复测的地图原点。按任务图取左下为 `field (0,0)`，+X 向右，+Y 向上，+Z 离开地面，朝 +X 的航向为 0°，逆时针为正。在车上标出四轮几何中心的地面投影，所有点和结果均以这个点计。`bounds_m=[xmin,ymin,xmax,ymax]` 限定车心和目标。模板为 `[0,0,2,2]`，可收窄为车心允许区。围挡标签的世界坐标仍以同一原点量取。
2. 用卷尺量停靠区四个角，按**米**填写 `docks_m`。若目标指的是车轮、车头或框架边线对准胶带，先换算成相应的车心位置。给车体半宽、转动时的四角扫掠、探头与围挡净空以及停车距离留足空间。
3. 每张墙标贴平、竖直、印刷顶边向上，码面朝场内，四周可见白边。读实际家族与 ID。在 `wall_tags` 逐张填写 `center_mm=[X,Y,Z]`，Z 为黑框中心到地面的垂直高度。`side_mm` 量**黑框外缘**的边长，不含白边和纸张边缘。`right_yaw_deg` 表示面对码面时从标签左边到右边的水平向量相对场地 +X 的逆时针角，区别于墙面法向。新场地可改家族、ID、数量和安装位置。每个 ID 唯一。
4. 在地上另贴几个复核点，记录位置和车头方向。选取至少两面墙都能在整圈内依次看到的位置，供第 4、7 节独立核对。

| 图上对象 | 模板场地位置 | 模板朝向或用途 |
| --- | --- | --- |
| A、B | (0.5, 1.5)、(1.5, 1.5) m | 停靠区左上、右上 |
| C、D | (1.5, 0.5)、(0.5, 0.5) m | 停靠区右下、左下 |
| 1 上墙、2 下墙 | (1000, 2000)、(1000, 0) mm | 打印右向分别 0°、180° |
| 3 左墙、4 右墙 | (0, 1000)、(2000, 1000) mm | 打印右向分别 +90°、−90° |
| 四墙标签 | 图示黑框边长 200 mm | 各张中心高度均需填写 |

**选择起点模式。** `start` 系的零点总是本次导航会话启动时的车心，当时车头为 +X、向左为 +Y，之后固定在地面。场地与启动系的关系用 `nav.json` 的 `start_field_pose` 指定。

- 初始放置任意，填 `null`。第一次给目标时，小车先原地多转一圈，用墙标把起点和初始车头方向注册到场地，随后惯性粗走。目的地的整圈扫描仍会执行。开机位置必须允许安全旋转并看到足够的墙标。
- 每次从同一标线启动，可填 `[场地X米,场地Y米,车头朝向度]`，实际把车摆到该位置和方向。跳过起点扫描。仅作换算示例：起姿 `[0.4,0.4,90]` 时，场地 A `(0.5,1.5)` 对应启动系 `(1.10,−0.10)`。车辆中途重启时重新量起姿，或改用任意起点模式。

## 3. 标定前摄内参与安装位姿

固定前摄支架、焦距和部署用分辨率。先运行 `v4l2-ctl -d "$FRONT_CAM" --list-formats-ext` 确认支持的宽高。在相同采集模式拍 12–20 张清晰的**普通黑白相机标定棋盘**，覆盖画面中央、四角、不同距离和倾角；固定平面和镜头，逐张移动标定板。九宫格任务板上的九个格子不适合作为下面的 9×6 内角点样板。

以下现场示例使用实际相机取**原图**。在树莓派交互终端输入已选的像素宽、高，按回车保存各姿态，输入 `q` 结束。选择一个空的 `lens` 目录用于本轮采集。

~~~bash
mkdir -p "$NAVCFG/lens"
read -r -p "前摄工作分辨率（宽 高） " W H
export W H
python - <<'PY'
from contextlib import closing
from pathlib import Path
import os
import cv2
from vision.camera import Mount, frames

dest = Path(os.environ["NAVCFG"]) / "lens"
size = (int(os.environ["W"]), int(os.environ["H"]))
with open("/dev/tty") as keyboard, closing(frames(os.environ["FRONT_CAM"], Mount(image_size=size))) as camera:
    for i in range(1, 31):
        print(f"摆好第 {i} 个姿态后回车，q 结束", flush=True)
        if keyboard.readline().strip().lower() == "q":
            break
        # 跳过移动标定板期间可能已进入摄像头缓冲的旧帧
        for _ in range(3):
            _, image = next(camera)
        path = dest / f"lens-{i:02d}.png"
        if path.exists() or not cv2.imwrite(str(path), image):
            raise RuntimeError(f"无法保存新图：{path}")
PY
~~~

按打印板修改内角点列数、行数和实际格宽。下面脚本示例按 **9 列 × 6 行内角点**，格宽使用操作员的毫米实测数。仅在识别到足够视角后将 `image_size`、`camera_matrix`、`distortion` 写入本车的 `front.json`。保留该文件已有的安装外参。RMS 单位为像素，保存前检查各帧残差，个别图显著偏大时检查模糊、弯板、误检后重拍。

~~~bash
python - <<'PY'
import json, os
from pathlib import Path
import cv2
import numpy as np

base = Path(os.environ["NAVCFG"])
cols, rows = 9, 6
# 从交互终端读取本次打印板的实测格宽
with open("/dev/tty") as tty:
    print("打印板格宽（实测毫米）", flush=True)
    pitch_mm = float(tty.readline())
if pitch_mm <= 0:
    raise ValueError("格宽应大于零")
pattern = np.zeros((cols * rows, 3), np.float32)
pattern[:, :2] = np.mgrid[:cols, :rows].T.reshape(-1, 2) * pitch_mm
world, images, names, size = [], [], [], None
for path in sorted((base / "lens").glob("*.png")):
    gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise ValueError(path)
    current = gray.shape[::-1]
    if size is not None and current != size:
        raise ValueError("图像分辨率不一致")
    size = current
    ok, points = cv2.findChessboardCornersSB(gray, (cols, rows))
    if ok:
        world.append(pattern)
        images.append(points)
        names.append(path.name)
if len(images) < 12:
    raise ValueError(f"仅 {len(images)} 张识别成功，请补拍")
rms, K, D, rotations, translations = cv2.calibrateCamera(world, images, size, None, None)
print("全局 RMS(px)", rms)
for name, uv, r, t in zip(names, images, rotations, translations):
    pred = cv2.projectPoints(pattern, r, t, K, D)[0]
    print(name, float(np.sqrt(np.mean(np.sum((uv - pred)**2, axis=2)))))
config = base / "front.json"
data = json.loads(config.read_text())
with open("/dev/tty") as tty:
    print("接受本次结果并保存？输入 yes", flush=True)
    if tty.readline().strip() == "yes":
        data.update(image_size=list(size), camera_matrix=K.tolist(), distortion=D.ravel().tolist())
        config.write_text(json.dumps(data, indent=2) + "\n")
PY
~~~

量前摄相对车心的**外参**。`front_xyz_mm=[前,左,上]` 为从车心地面投影指向光心的位移，向后、向右取负，最后一项为光心离地高度。可用直尺和铅垂线将光心投到地面后分别量取。`front_ypr_deg=[偏航,仰角,滚转]` 描述相机相对车体的安装，镜头水平朝车头且图像朝上时三角为零。镜头向左偏航为正，向上仰为正，滚转绕前轴按右手定则（正向使左侧上抬）。用场地水平线和带垂线的竖板核对图像方向。这里与 `field.json` 的标签尺寸、车心使用同一实体基准。

## 4. 先用静止车核对视觉与 IMU

~~~bash
python -m vision.scan front --source "$FRONT_CAM" --field "$NAVCFG/field.json" --frames 15
python -m vision.front --source "$FRONT_CAM" --config "$NAVCFG/front.json" \
  --field "$NAVCFG/field.json" --frames 30
python -m move.imu --port "$IMU_PORT" --origin 0 0 0 --mount-yaw 0
~~~

在已独立量好的两个以上车心点分别朝向不同墙标，逐次执行前两条。首条确认家族、ID 和角点，第二条读取 `pose.x_m/y_m/yaw_rad/tag_ids/rms_px`。将车心、朝向对照地面刻度，从不同墙标得到的一致偏差先查车心和光心偏移，单张墙不符先查其 ID、中心、黑框尺寸及打印右向。只看到像素但 `pose:null` 时检查内参、畸变、标签正反方向和相机高度。程序会舍弃镜像歧义、越过重投影/高度/倾斜门限或互相冲突的观测。像素 RMS 很低也可能存在共同的地图偏移，最终仍以标尺复核。

IMU 固定在车架上，传感器接近水平、+Z 朝上，量 IMU +X 相对车头的逆时针夹角，写入 `imu_mount_yaw_deg`，并将上例 `--mount-yaw 0` 换为同一实测角度。端口为 115200，输出原始 `0x04` 和四元数 `0x16` 或欧拉角 `0x26`。启动后先静止，之后用车头方向标线核对左转时航向增加、向前短移主要增加启动系 X、向左短移主要增加 Y。航向跨 ±π 时显示跳变属于角度表示。确认后退出独立 IMU 和相机进程。静止校准期间不要推车、拿起车架或转舵机。[IMU 输出与记录](imu/README.md)

## 5. 核对避障并量六向速率

预热 Uno 和探头后，先停车检查地址，再观察连续读数。让平整挡板分别靠近各探头，对照车身前后左右和返回方向。退出 `--watch` 后再运行其他 Uno 命令。

~~~bash
python -m move stop
python -m move.sonar scan
python -m move.sonar read --watch
~~~

| 车身方向 | 前 | 后 | 左 | 右 |
| --- | --- | --- | --- | --- |
| 七位 I²C 地址 | 0x69 | 0x72 | 0x6C | 0x6A |

探头头部到障碍面应有稳定回波，四向均须**大于 300 mm**（当前 `sonar::STOP_MM`），读数还须在 650 ms 内。四路顺序轮询正常约 432 ms。一轮扫描、通信延迟和车辆惯性均会影响实际制动距离。实测速度下，按车体突出部分留净空；若余量不足，先增大固件停车距离、更新固件并复核。墙面过斜或射线打不到围挡可能返回 `null`，也会锁停。选择各朝向都能获得可靠回波的区域。用挡板操作时保持人员离开车辆运动和旋转范围。

在实际地面、电池和载荷条件下，按车心在地面标两个坐标轴，再在车体和地面作初始方向标记。先架空核对六种动作的轮向，然后移至有充分净空的地面。每次只发**一个**短时手动动作，等完全静止后量主方向位移或实际转角。示例先输入选定的动作和短时毫秒数，多组测量均记录起停位置；一次只占用一个 Uno 端口。

~~~bash
read -r -p "动作（go/back/left/right/rotate-left/rotate-right）及脉冲毫秒数 " ACT MS
python -m move "$ACT" --ms "$MS" --port "$UNO_PORT"
~~~

| `speeds` 键 | 实测量 | 写入的正值 |
| --- | --- | --- |
| `go`、`back` | 地面车心沿前后轴的位移绝对值，各量 | 位移 m ÷ (命令 ms/1000) |
| `left`、`right` | 地面车心沿左右轴的位移绝对值，各量 | 位移 m ÷ (命令 ms/1000) |
| `rotate-left`、`rotate-right` | 停稳后车头相对起始方向的实际角度，各量 | 角度绝对值 ° ÷ (命令 ms/1000) |

每个方向取多次典型脉冲的有效速率，分别填写 `nav.json.speeds`。特别记录能克服静摩擦的最短脉冲、单次最小落点位移、转动时车心是否平移、以及命令结束到完全停止的时间。用这些数据设 `min_pulse_ms`、`max_pulse_ms`、`settle_s`。单次最小位移如果已经大于目标容差，先处理地面、传动或执行步幅。控制器将实测速率用于估算点动时长，位移和转角由实时传感器更新。

## 6. 调整控制参数

以下是 `nav.json.control` 默认值。全部为正数，脉冲毫秒数、帧数、标签数和停滞次数取整数。与现场允许误差直接相关的是 `tolerance_m`。

| 字段 | 默认值 | 含义与调整依据 |
| --- | --- | --- |
| `coarse_m` / `tolerance_m` | 0.18 / 0.035 m | 距目标进入整圈扫描的惯性距离、最终每一帧允许的视觉距离。后者必须小于前者。 |
| `min_pulse_ms` / `max_pulse_ms` | 65 / 350 ms | 单次动作时长上下限。最小不大于最大，最大不超过 1000 ms。按六向实测最小可靠步幅和制动距离设定。 |
| `settle_s` | 0.25 s | Uno 确认动作结束后等待惯性沉降，之后恢复零速约束并使用新相机帧。 |
| `scan_step_deg` | 18° | 每次扫描点动的角度上限，最大 45°。窄视场或漏过墙标时减小，仍按 IMU 累计左转满 360°。 |
| `min_scan_tags` | 2 | 一圈共识至少需要的不同墙标 ID 数，应不超过该工位实际可见的已建图标签数。 |
| `confirm_frames` | 3 | 扫描共识所需观测数及停车精调要求的连续新帧数，至少 2。 |
| `agreement_m` / `agreement_deg` | 0.12 m / 12° | 跨帧或跨朝向的位姿一致性门限，角度小于 90°。先解决地图和镜头系统误差。 |
| `vision_timeout_s` | 3 s | 在一次驻停中等待有效且一致的新观测的预算。完全失去相机帧会更早中止。 |
| `timeout_s` | 150 s | **每个目标**的总时限，包含任意起点的首次注册、粗走、整圈、回看与微调。 |
| `stall_pulses` | 6 | 连续若干次控制无法取得足够净进展后终止，检查方向和最小步幅。 |
| `calibration_s` | 2 s | 打开会话时 IMU 的静止采样时间。每个新会话重做。 |

## 7. 现场运行与验收

### 绕板与棋盘作业位

`nav.json` 的 `board_approach` 以 30 cm 方板、34 cm × 34 cm 底盘、南侧 50 cm 预靠点为初始设置。板与格线应同心同向，地图的 `board_origin_mm` 仍填写 20 cm 格线的左下角。确认轮子、支架与收拢的机械臂的实际外廓，把制动距离和定位余量计入 `clearance_m`。现有配置加入这个区块后，A/B/C/D 和坐标导航也绕开固定板。原地扫标签前需要留出车角转动净空。

从 `Agents` 运行离线规划。未填实测 `work_distance_m` 时，预览只给出预靠路径，作业位为 `null`。

```sh
python -m move.navigation --board --dry-run --from-field 1.0 1.6
```

先手摆测试车心距棋盘中心 50、45 和 40 cm 时的净空、角标识别和机械臂可达性。将确认的车心距离（米）填入 `work_distance_m`。默认 `heading_deg=90` 使车头指向棋盘。如果从车身侧面作业，按实际车头方向选 0 或 180，最后一段会按车体系换算为横移。用已知车心及朝向通过 `vision.robot --calibrate … --save-mount …` 标定下视相机。前摄和下视必须使用不同设备。

```sh
python -m move.navigation --config "$NAVCFG/nav.json" --board \
  --top-config "$CFG/rig.json" --top-source /dev/video2
```

到预靠点后才调航向。最后以短步平移并停车取新帧，稳定观测到作业位才返回到达结果。已建立棋盘观测后失标会停车退出。同一导航会话从作业位发起下一次坐标任务时，先保持航向退回预靠点。

### 停靠点与坐标验收

先让车心和车头精确对好地面标线。选择有净空的短程坐标，之后逐个 A、B、C、D，最后连续运行。以下每条 CLI 命令会新建启动零点。使用已知起点模式时，每次独立启动都要使车体起姿与配置一致。需要保留同一零点时，用一次 `--dock A B C D` 或同一个 Python 会话。

~~~bash
python -m move.navigation --config "$NAVCFG/nav.json" --xy 0.25 0 --frame start
python -m move.navigation --config "$NAVCFG/nav.json" --dock A B C D
python -m move.navigation --config "$NAVCFG/nav.json" --xy 1.05 0.95 --frame field
~~~

1. **粗走。** 看车体是否向目标方向移动。每次短步后停车，IMU 更新剩余位移。若起姿未预设，会先看到一次额外的起点原地扫描。
2. **整圈。** 进入粗定位半径后，车体原地左转，按实测航向累计至少 360°。途中分步停车取帧，将各次墙标观测对齐至同一 IMU 时间基准。程序取多角度一致结果，再转回一个观测质量好的方向。
3. **微调和到达。** 每次只朝车身前后或左右的一个方向点动。停车后须取得新图像，同时检查 IMU 和四路超声。最终连续若干帧的车心都落入目标容差后才输出 `Arrival` JSON。`error_m` 是末帧视觉误差，`field_yaw_rad` 是实测航向，任务只约束位置。没有标签、回执中断或长期无进展会退出。
4. **遇障锁停。** 在低速、有余量的直线路段及原地扫描期间，用挡板从侧面进入可测范围，观察四轮在新测量触发后停车。移走挡板，确认剩余停靠点不继续执行。只有操作员排除原因、确认车体停稳且四面读数恢复后，才运行下面的解锁命令；然后按当前起姿发起一项新任务。`STOP` 和蓝牙重连均不解除障碍锁存。锁存在 Uno RAM 中，断电后重新确认四向并重新建系。
5. **独立验收。** 每次由标尺/铅垂标记量最终车心在两轴的误差，计算 `sqrt(Δx²+Δy²)`，与视觉结果分别记录。同一目标从不同方向、起点和航向重复。以独立量得的位置误差判断能否满足现场任务，软件默认 35 mm 是判定设置。

~~~bash
python -m move.navigation --reset-lock --uno-port "$UNO_PORT"
~~~

| 初始车心 / 航向 | 目标系 / 点 | 扫描见到的墙标 | 末帧位置和 `error_m` | 标尺 Δx、Δy / m | 锁停方向或失败阶段 |
| --- | --- | --- | --- | --- | --- |
| | A | | | | |
| | B | | | | |
| | C | | | | |
| | D | | | | |
| | 任意坐标 | | | | |

| 现场现象 | 首先核对 |
| --- | --- |
| 手动可走，导航返回 `ERR ACTION/DURATION` 或 `ERR NAV INACTIVE` | 核对所烧固件版本、Uno 是否在任务中复位，以及电源与 HC-05 链路。恢复后重新建系。 |
| `ERR OBSTACLE 方向 毫米` 或 `ERR SONAR 方向` | 前者是该探头测得阻挡，后者是缺测/过期。停车观察四向读数、分线板和标签方向，排除后人工解锁。 |
| 一圈后没有足够定位共识 | 用只读 `vision.front` 逐面检查编号、地图、打印右向、实收分辨率、曝光和安装角，再检查正转方向、视场与步长。 |
| 圈数不符、粗走偏或反复报无进展 | 对照地面标线检查 IMU 轴向和安装、六个动作的真实方向、蓄电和打滑、停稳时间及起点配准。 |
| 视觉输出接近目标而实物总偏同一方向 | 比较不同墙标和车头朝向，分别复核地图原点、车心到光心偏移、黑框边长、目标是车心还是车体边线。 |
