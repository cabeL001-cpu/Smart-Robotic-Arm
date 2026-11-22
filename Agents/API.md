# 系统接口手册

从仓库的 `Agents/` 目录运行以下命令或导入 Python 模块。`MHS` 是动作入口。`vision` 给出测量结果，`move.imu` 给出惯性状态，`listen_speak` 负责收发语音。首次部署和实车标定分别见[部署指南](DEPLOY_RASPBERRY_PI.md)、[小车标定](move/CALIBRATION.md)和[机械臂标定](arm/CALIBRATION.md)。

## 1. 坐标导航和任务分派

~~~python
from pathlib import Path
import MHS

cfg = Path.home() / ".config/smart-robotic-arm/navigation/nav.json"
with MHS.open_navigation(cfg) as nav:
    a = nav.go_to_dock("A")
    print(a.field_x_m, a.field_y_m, a.error_m)
    b = MHS.dispatch(("B", "arrive"), navigator=nav)
    # 在同一个启动系中，前往距原点前方 0.40 m、左方 0.10 m 的坐标
    point = nav.go_to(0.40, 0.10)
~~~

这里的目标是**车心在地面上的投影**。`start` 系原点是本次启动静止时的车心，+X 沿当时车头，+Y 向左。车辆转向后启动系仍固定在地面。`field` 系以标定地图为准，图示原点为场地左下角，+X 向右、+Y 向上。两者都用米，正航向为逆时针。

| 调用 | 参数、行为和返回 |
| --- | --- |
| `MHS.open_navigation(config, *, uno_port=None, imu_port=None, front_source=None)` | 上下文管理器。`config` 是导航 JSON 文件路径。`uno_port` 默认为 `UNO_PORT` 或 `/dev/rfcomm0`；`imu_port` 默认为 `IMU_PORT`，115200 波特；`front_source` 默认为 `FRONT_CAM`，接受实时相机编号或 V4L2 节点。打开串口后先停车，读取静止 IMU 并确认前摄产生新帧，再进入 Uno 四向保护模式。打开本身不会驱动车轮。一次上下文内顺序执行多个目标，共用启动零点。 |
| `nav.go_to_dock(name)` | `name` 为 `field.json` 中已测量的停靠点键，输入转为大写。A、B、C、D 的值是场地系米坐标。返回 `Arrival`。 |
| `nav.go_to(x_m, y_m, *, frame="start")` | `x_m/y_m` 是**目标位置**，不是相对当前位置的增量。`frame="field"` 直接输入地图位置。惯性粗走后按 IMU 实测左转满一圈，在不同朝向识别墙标，然后停车取新帧、短步微调。越界、定位失败或阻挡均不会返回到达结果。 |
| `MHS.go_to(x_m, y_m, *, config, frame="start", uno_port=None, imu_port=None, front_source=None)` | 单目标快捷函数，参数同上。每次调用都会重新建立启动原点；多段行程请使用同一个 `nav`。 |
| `nav.shared_link()` | 上下文管理器。两次导航之间独占复用 Uno，例如向已停车的机械臂工位发关节指令。进入、退出均检查导航保护。与 `go_to` 并发使用会抛出 `RuntimeError`。 |
| `MHS.dispatch(command, *, navigator=None, arm_config=None, link=None)` | `command=(target, action)`。`(None,None)` 返回 `None`。`("A","arrive")` 等字母调用给定 `navigator`，返回 `Arrival`。`("4","arrive")` 等格号调用机械臂，须给实测 `ArmConfig`，返回两关节角。格号操作要求车已停在机械臂的标定工位。可传同一 `link`；提供 `navigator` 时自动复用其连接。未知目标或动作报 `ValueError`。 |
| `move.navigation.reset_navigation_lock(*, uno_port=None)` | 人工确认四面已畅通后调用。停车握手并请求解锁，成功返回 `None`。不会重发原任务或驱动车轮。 |

地图坐标变换可以离线调用。`move.nav_config.NavigationConfig.load(path)` 解析相对配置路径、实测速率和 `Control` 参数；`Origin(x_m,y_m,yaw_rad)` 表示启动系原点在场地中的位姿。`origin.to_field(x,y)` 和 `origin.to_start(x,y)` 分别将米坐标从启动系转换至场地系及反向转换。`Origin.match(x_field,y_field,yaw_field,odom)` 由同一时刻的场地观测和 IMU `State` 配准两系，航向输入均为弧度。[控制参数逐项定义](move/CALIBRATION.md#6-调整控制参数)

`Arrival` 包含 `goal_field_m=(x,y)`、末帧视觉车心 `field_x_m/field_y_m`、场地航向 `field_yaw_rad`、同一点在启动系的 `start_x_m/start_y_m`、末帧到目标的距离 `error_m` 和最终确认中使用过的 `tag_ids`。只有全部连续确认帧都在 `tolerance_m` 内才返回。CLI 将这些字段输出为 JSON。实物误差还受标尺、镜头和车心标定误差影响。

### 实时采集接口

直接集成采集层时从 `move.nav_sensors` 导入以下函数。`stamp`、`deadline` 都是本机 `time.monotonic()` 时间轴上的**绝对秒数**。例如等待两秒传 `deadline=time.monotonic()+2`。摄像头使用主机完成取帧的时间，IMU 使用主机收到串口数据并扣除同批后续字节传输时间的时间。导航由同一进程读取二者。

| 接口 | 调用与结果 |
| --- | --- |
| `open_imu(port,calibration_s,mount_yaw_deg)` | `with open_imu(...) as imu` 打开 115200 串口并做静止校准。`imu.snapshot()` 返回 `(State,累计有符号转角弧度)`，累计转角不受视觉修改航向的影响。`imu.at(stamp,deadline)` 等待并插值得到曝光时刻状态。超过采样与历史时限会抛 `TimeoutError`。 |
| `imu.moving()`、`imu.settle(seconds,deadline)`、`imu.correct(stamp,x,y,yaw)` | 分别从下一个采样起解除零速约束、在制动后等待沉降再开始零速约束（返回沉降截止时刻）、用启动系米坐标与弧度校正历史。`correct` 成功返回 `None`，过期或重复观测抛 `TimeoutError`。 |
| `open_front(source,mount)` | `with open_front(...) as camera` 持续获取实时前摄新帧，`source` 为相机号或设备路径，`mount` 为这台相机的 `Mount`。`camera.after(stamp,deadline)` 返回 `(新时间,已去畸变帧)`，`camera.check_live()` 检查采集故障或过期。 |

### 命令行

~~~bash
python -m move.navigation --config "$CFG/navigation/nav.json" --dock A B C D
python -m move.navigation --config "$CFG/navigation/nav.json" --xy 1.05 0.95 --frame field
python -m move.navigation --config "$CFG/navigation/nav.json" --xy 0.40 0.10 --frame start
python -m move.navigation --reset-lock --uno-port "$UNO_PORT"
~~~

`--dock`、`--xy`、`--board`、`--reset-lock` 四选一。`--uno-port`、`--imu-port`、`--front-source` 可覆盖设备环境变量。默认 `--frame start` 只解释 `--xy`。每条独立 CLI 命令均新建会话；已知起点模式应在配置给定的起姿开机。`--dock A B C D` 在一次会话中执行，首个失败就退出。`MotionBlocked` 是 `OSError` 子类，含第一个阻挡方向；超时为 `TimeoutError`，无法形成定位共识、无进展为 `RuntimeError`，标定或坐标错误为 `ValueError`。见[地图、运动参数、容差和恢复流程](move/CALIBRATION.md)。

`nav.go_to_board(top_config=..., top_source=...)` 读取 `nav.json` 的 `board_approach` 参数，绕到预靠点并朝向棋盘作业侧，最后返回下视相机确认的 `Arrival`。CLI 使用 `--board --top-config ... --top-source ...`，可用 `TOP_CAM` 提供默认设备。进入工位后可通过 `nav.shared_link()` 顺序操作机械臂。

## 2. 手动底盘、测距与共享串口

| 接口 | 调用约定 |
| --- | --- |
| `MHS.open_uno(port=DEFAULT_PORT)` | `with MHS.open_uno() as link: ...`。以 9600、8N1 独占打开 Uno 串行端点，重发 `STOP` 至 `OK STOP`，退出时关闭。`DEFAULT_PORT` 在导入时读取 `UNO_PORT`，缺省为 `/dev/rfcomm0`。 |
| `MHS.move(action, duration_ms=1000, *, port=DEFAULT_PORT, link=None, guarded=False)` | `action` 见下表。`duration_ms` 为 1–65535 的整数毫秒，`stop` 忽略时长。`link` 可传已握手连接，否则自行打开 `port`。成功停车后返回 `None`。`guarded=True` 用于已处于 `NAV BEGIN` 的会话，向固件发送带导航前缀的动作。 |
| `move.encode_command(action, duration_ms=1000)` | 只做动作和时长校验，返回带换行的 ASCII `bytes`，例如 `b"GO 250\n"`。 |
| `move.sonar.scan(link)` | 停车时扫描四个预设七位地址，返回实际应答的 `list[int]`。顺序为前 0x69、后 0x72、左 0x6C、右 0x6A。 |
| `move.sonar.read(link)` | 返回 `front/back/left/right` 字典，每项形如 `{"address":"0x69","mm":412}`。缺测和过期时 `mm=None`，没有有效距离不表示方向畅通。 |
| `uno.stop_wheels(link)`、`uno.wait_reply(link, expected, seconds=1, *, fields=False)` | 前者发停车指令并等待确认。后者等待 `expected` 字节串；普通成功返回 `None`，`fields=True` 时接受同前缀和空格分隔的字段并返回完整行。阻挡、意外回执、超时均抛出异常。 |

| 动作名 | 含义 |
| --- | --- |
| `go`、`back`、`left`、`right` | 车体系前进、后退、左移、右移 |
| `forward-left`、`forward-right`、`back-left`、`back-right` | 向相应两方向斜移 |
| `rotate-left`、`rotate-right` | 原地左旋、右旋 |
| `turn-right-left-wheels`、`turn-right-front-wheels` | 分别以左侧轮、前轮执行原固件的右转动作 |
| `m1`、`m2`、`m3`、`m4` | 对应扩展板电机座单轮正转 |
| `stop` | 四轮停车。保持舵机已有输出，保留导航锁存 |

从 `Agents` 运行 `python -m move ACTION [--ms 毫秒] [--port 端口] [--dry-run]`。`--dry-run` 只打印指令。`python -m move.go_one_second` 实际请求前进 1000 ms。`python -m move.sonar scan` 扫描，`python -m move.sonar read [--watch] [--port 端口]` 读取；`--watch` 每 0.25 秒打印一次。`sh bt_link.sh bind [MAC或模块名]` 建立蓝牙串口节点，`status` 查状态，`release` 释放绑定。可设 `BT_NAME`、`BT_DEV`、`BT_CHANNEL`，默认分别为 `sjtu006`、`rfcomm0`、`1`。[底盘协议和接线](move/README.md) · [蓝牙配置](BLUETOOTH_WORKFLOW.md)

### Uno 线上协议

每条指令是以换行结束的 ASCII。动作大写，时长是十进制毫秒。新导航必须使用包含 `NAV` 命令的完整 `move` 固件。

| 请求 | 正常响应 | 说明 |
| --- | --- | --- |
| `STOP` | `OK STOP` | 所有模式均可停车，停车不解锁。 |
| `GO 250` 等手动动作 | `OK MOVE`，自然到时 `DONE` | 手动模式按动作所需方向检查；已锁存时所有车轮动作被拒绝。 |
| `SONAR`、`SONAR SCAN` | `SONAR 0x69=... ...`、`SCAN ...` | 快照中 `NA` 为无有效回波。地址扫描仅在停车时执行。 |
| `ARM 肩脉宽 肘脉宽` | `OK ARM` | 微秒整数。缺标定返回 `ERR ARM CALIBRATION`，越界返回 `ERR ARM`，两路都通过才更新。 |
| `NAV BEGIN`、`NAV CHECK` | `OK NAV BEGIN`、`OK NAV CHECK` | 前者在停车时布防，后者确认保护仍在。整段会话含停车等待期间均检查四面。 |
| `NAV MOVE GO 200` 等 | `OK MOVE` 后 `DONE`，或阻挡错误 | 仅接受已布防会话，Uno 复位后收到旧动作会回复 `ERR NAV INACTIVE`。 |
| `NAV END`、`NAV RESET` | `OK NAV END`、`OK NAV RESET` | END 退出正常会话，不能清除锁存。RESET 在停车且四路有效畅通时解除锁存并回到手动待命。 |

`ERR OBSTACLE FRONT 240` 表示该探头读到 240 mm；`ERR SONAR FRONT` 表示缺测或过期。运动中触发时固件先停车，再返回错误，本动作没有 `DONE`。导航保护期间锁存首次原因，原处等待或重连都不会自行继续。普通手动模式仍保留按方向退离的用法。`ERR BUSY` 表示命令与当前动作冲突；`ERR ACTION/DURATION/LINE` 分别表示动作、时长或输入行无效。

## 3. 机械臂

| Python 接口 | 参数和结果 |
| --- | --- |
| `ArmConfig.load(path)`、`ArmConfig(geometry, shoulder, elbow)`（均在 `arm`） | 从 JSON 加载或以对象构造。`geometry` 为 `Geometry`，两轴为 `ServoCalibration`，均有默认值。默认机械端点留空，驱动前须实测。 |
| `MHS.arm(cell, *, config=ArmConfig(), port=DEFAULT_PORT, link=None, previous=(0,0))` | `cell` 为 0–8 的整数或单字符格号，从左下起逐行编号。`previous` 为上次肩／肘角（度）用于选择最近可行逆解。发送指令并返回已接受的 `(肩角,肘角)`；到位由视觉测量。 |
| `arm.set_joints(angles, *, config=ArmConfig(), port=DEFAULT_PORT, link=None)`、`arm.joint_command(angles, config)` | `angles=(肩角,肘角)`，单位度。前者按实测限位下发并等待确认，成功返回 `None`。后者只生成 `ARM ...\n` 字节。 |
| `cfg.axes()`、`cfg.joint_limits()`、`cfg.solve(x_mm,y_mm,previous=(0,0),geometry=None)` | 依次返回每轴的标定和图纸角限、两轴可用角度区间、给定平面目标的两关节角。`geometry` 可替换本次停靠工位的基座。不可达抛 `ValueError`。 |
| `cfg.write_firmware_limits(path)` | 从同一标定生成 Uno 的 `arm_limits.h`，两轴均有效后替换文件。随后编译并上传才能改变板上限位。 |
| `ServoCalibration.pulse_limits(design_limit=inf)`、`angle_limits(design_limit)`、`pulse(angle_deg, design_limit=inf)` | 将实物行程、两端余量、图纸角限和电气限位求交，分别返回整数微秒区间、角度区间、一个合规整数脉宽。各字段的物理定义和测法见标定手册。 |
| `arm.cell_center(cell)`、`forward(shoulder_deg,elbow_deg,geometry=Geometry())`、`inverse(x_mm,y_mm,geometry=Geometry(),previous=(0,0),*,joint_limits=None)` | 前两者返回图纸框架坐标（毫米）。逆解返回两关节角，可选 `joint_limits` 为两轴各自的 `(最小角,最大角)`。`vision.arm.target_xy(cell)` 返回棋盘左下为零的毫米坐标，图纸中的棋盘左下为 `(50,50)`。 |

CLI 为 `python -m arm`，`--cell 格号`、`--joints 肩角 肘角`、`--export-limits 头文件` 三选一。用 `--config 文件` 加载实测 JSON，`--port` 选 Uno 端口，`--dry-run` 只计算。带载使用前先完成[关节与顶摄标定](arm/CALIBRATION.md)。

`Geometry` 的 `upper_mm/forearm_mm` 是两段平面长度；`base_x_mm/base_y_mm/base_yaw_deg` 是同一工位中肩轴的位置和基座朝向；`shoulder_limit_deg/elbow_limit_deg` 是两轴图纸对称角度上限。每个 `ServoCalibration` 单独填写 `zero_us`（几何零角截距）、`us_per_degree`（有符号斜率）、`min_us/max_us`（可用电气边界）、`angle_at_max_deg`（最大机械位置的几何角）、`retreat_sign`（从该位置退回时角度变化的符号）、`margin_deg`（两端分别留出的余量）。

## 4. 视觉与场地地图

CLI 按 `python -m 模块名` 调用。视觉输入 `--source` 可为十进制相机编号、V4L2 节点、图片或录像；真正下发动作时要求实时设备。`--config` 是对应镜头的标定，`--field` 是场地地图。JSON 行中的 `t` 是实时取帧的单调时刻（秒），录像使用录像时间，单张图片为零。`pose: null` 表示本帧没有合格观测。

| 模块 | 参数与输出 |
| --- | --- |
| `vision.scan` | 位置参数 `top` 或 `front`，必填 `--source`，可选 `--config`、`--field`、正整数 `--frames`。输出标签家族、ID、中心及四角像素。顶摄还输出 `board_tags_used`、`board_rms_px`。 |
| `vision.front` | `--source`、`--config front.json`、`--field field.json` 必填，`--frames N` 可限帧。使用镜头内外参和墙标实测三维位置。输出 `pose.x_m/y_m/yaw_rad/tag_ids/rms_px`，适合独立标定前摄及作为绝对观测。 |
| `vision.arm` | `--cell 0..8` 必填，`--source` 默认 `0`，`--config` 可选。`measurement` 含 `tip_mm, target_mm, error_mm, tags_used, rms_px`，`arrived` 按连续两秒判定。 |
| `vision.align` | `--save-mount 输出JSON --base X毫米 Y毫米 朝向度` 标定基座，或 `--cell 格号 --arm-config 关节JSON --port 端口` 实时闭环。两种模式都使用 `--source`、`--config`；前者可用静态图，后者会驱动舵机。 |
| `vision.robot` | `--source` 默认 `0`，`--config` 为顶摄车心标定，`--field` 为场地配置。`--calibrate X米 Y米 朝向度 --save-mount 文件` 成对使用，保存已知姿态下车心投影。平时输出 `pose`；加 `--cell N` 同帧输出 `arm`。 |

主要可组合的视觉接口如下。`tags` 为 `{(family,id): 像素角点[4,2]}`，角点按码图左上、右上、右下、左下排列。`size=(宽,高)`。

| Python 接口 | 用途 |
| --- | --- |
| `vision.camera.Mount.load(path=None)`、`mount.save(path)`、`mount.rectify(frame)`、`frames(source,mount)`、`live_source(source)` | 加载／保存 JSON，检查分辨率、去畸变；帧生成器依次给出 `(t,frame)`，`live_source` 判断实时来源。`image_size,camera_matrix,distortion` 属于镜头；`height_mm,nadir_px,tip_height_mm,tip_offset_mm,tip_family,tip_id` 用于顶摄末端；`arm_base_px,arm_x_px,robot_center_px,robot_forward_px` 为标定得到的投影点；`front_xyz_mm,front_ypr_deg` 属于前摄安装外参。 |
| `vision.tags.Detector(families=("36h11","36h10"), *, board=True).detect(frame)` | 返回 `tags`。`board=True` 启用印刷棋盘补识别；前摄墙标使用 `board=False`。`vision.scan.inspect(tags,role,field)` 生成对应角色的扫描记录。 |
| `vision.printed.printed_code(bits)`、`recover_neighbors(gray,tags)` | 前者接收 80×80 的黑白 0/1 候选块，返回 `(编号,四分之一圈旋转次数)` 或 `None`。后者在灰度图上利用已解出的两、三个实物棋盘角标查找其余角标，原地补入 `tags`。 |
| `vision.field.Field.load(path=None)`、`field.floor_layout()`、`field.wall_ids()`、`field.wall_layout()` | 场地可设置 `bounds_m,docks_m,wall_family,wall_tags`。下视还可配置 `board_origin_mm,board_yaw_deg,floor_tags`；旧地图可用 `wall_heights_mm,height_tolerance_mm`。布局函数返回地标二维角点、墙标 ID 集合、各墙标的三维中心／向右轴／边长。 |
| `vision.front.FrontLocalizer(mount,field,*,max_rms_px=2.5,max_ground_mm=80,max_tilt_deg=12).measure(frame)` | 输入已去畸变的前摄帧。用重投影均方根、车心地面高度残差和车体倾斜角排除平面歧义。返回 `FrontFix` 或 `None`。单位见参数后缀。独立 CLI 和导航默认使用表中的质量门限。 |
| `vision.board.square(x,y,side,yaw_deg=0)`、`project(matrix,points)`、`fit_plane(tags,layout=BOARD_LAYOUT,min_tags=2,tolerance_px=3)`、`axes(plane,origin_px,x_px)` | 生成方形角点、用单应投影、多标签拟合平面、由两个投影点求平面原点与朝向（度）。成功的 `Plane` 含 `to_pixel,to_world,tags_used,rms_px`，拟合失败返回 `None`。 |
| `vision.field.locate_floor(tags,mount,field)`、`wall_plane(tags,mount,field)`、`wall_views(tags,K,layout,family)` | 前者返回地面变换和来源 `floor/wall`，缺测为 `(None,None)`。第二个接口具有下视方向约束。`wall_views` 为两种相机共用的几何候选生成器，依次给出标签键、场地到相机旋转和平移、场地中的光心（毫米）、像素重投影误差。 |
| `vision.robot.calibrate(mount,plane,size,x_m,y_m,yaw_deg)`、`pose_from_plane(plane,source,mount)` | 在已知工位写入 `mount` 的车心与车前点；后者返回 `Pose(x_m,y_m,yaw_rad,source,tags_used,rms_px)` 或 `None`。 |
| `vision.arm.target_xy(cell)`、`measure(tags,plane,size,cell,mount)`、`Hold(seconds=2,max_gap=.5).update(t,observation)` | 计算棋盘格心，返回一次末端测量或 `None`，累计误差严格小于 6.7 mm 的连续观测。漏帧、时间倒退或帧间隔超过允许值会重新计时。 |
| `vision.align.observations(source,mount,cell)`、`align(cell,samples,send,mount,config,settle_s=.6,max_steps=10,timeout_s=20)` | 前者产出 `(t,plane,measurement)`；后者通过 `send((肩角,肘角))` 实施校正。参数依次控制沉降等待秒数、最大修正次数和观测超时。成功返回最后一次 `Measurement`，失败抛异常。 |

## 5. IMU 采集、融合与回放

~~~bash
python -m move.imu --port "$IMU_PORT" --origin 0 0 0 --mount-yaw 0 --record "$DATA/run-raw.csv"
python -m move.imu --replay "$DATA/run-raw.csv" --fixes "$DATA/poses.jsonl"
~~~

`--port` 与 `--replay` 必选其一。`--record` 写入新的原始样本 CSV。`--origin X米 Y米 朝向度` 默认全零，`--mount-yaw` 是 IMU +X 相对车头的逆时针角度。`--calibration-seconds` 默认 2，期间保持静止。`--stationary` 持续施加零速约束，适用于固定台架。`--vision-stdin` 接收同一主机同时采集的 `{"t":...,"pose":...}` JSON 行；`--fixes` 为回放同一时基的观测文件。这两项互斥。输出 `t,x_m,y_m,yaw_rad,vx_mps,vy_mps,vision_fixed`。导航会话自己占用 IMU 端口，独立采集程序应先退出。

| Python 接口 | 数据与参数 |
| --- | --- |
| `move.imu.sensor.Decoder().feed(data,received_at)` | `data` 是串口字节，`received_at` 为同机单调秒。产出配对的 `Sample(t,accel_g,gyro_rad_s,quat)`，四元数按 `w,x,y,z`，原始加速度用 g，角速度用 rad/s。 |
| `serial_samples(link)`、`read_csv(stream)`、`record_samples(samples,stream)`（在 `move.imu.sensor`） | 串口或 CSV 转样本迭代器；第三个原样转发样本并写 CSV。CSV 表头为 `time_s ax_g ay_g az_g gx_rad_s gy_rad_s gz_rad_s qw qx qy qz`。`euler_quaternion(roll,pitch,yaw)` 用弧度转相同顺序的四元数。 |
| `move.imu.Tracker(stationary_samples, *, x_m=0,y_m=0,yaw_rad=0,mount_yaw_rad=0)` | 用静止样本估计零偏与方向。Python 的两个角参数用弧度。`state` 为 `State(t,x_m,y_m,yaw_rad,vx_mps,vy_mps)`。 |
| `tracker.update(sample,stationary=False)`、`state_at(t)`、`correct(t,x_m,y_m,yaw_rad)` | 更新并返回状态；在最近一秒历史上插值，历史外返回 `None`；按观测时刻对齐绝对位姿并保留之后的相对运动，接受返回 `True`，过期／重复观测返回 `False`。`rotate(x,y,a)` 按弧度旋转平面向量，`wrap(yaw)` 归一化航向，`attitude(q)` 返回传感器到导航系的旋转矩阵。 |

## 6. 语音

~~~python
from listen_speak import listen, speak, say_event
command = listen(timeout=8)
if command[0] is not None:
    say_event("start", number=command[0])
~~~

| 接口 | 参数、结果及副作用 |
| --- | --- |
| `MHS.listen(timeout=None)`、`listen_speak.listen(timeout=None)` | `timeout` 是等待开始说话的秒数，缺省取配置。检查模型配置和网络后录音、本地转写、在线分类。正常返回 `("0"…"8","arrive")`，空指令或不可用时返回 `(None,None)` 并按情况播报。当前语音分类只产生数字格号。字母停靠指令可从调用方直接交给 `MHS.dispatch`。 |
| `listen_speak.asr.listen(timeout=None)` | 只录音与离线识别，返回无空格文本或空字符串。`listen_speak.llm.classify(text)` 单独解析文字，失败可抛 `LLMNotConfigured`、`LLMOffline` 或 `LLMParseError`。`llm.is_online()` 检查服务可达性。 |
| `MHS.say(text,offline=False)`、`listen_speak.speak(text,offline=False)` | 同步播放文本，优先使用缓存；在线可用 edge-tts，离线尝试 Piper，再回退 espeak-ng。成功返回 `True`，底层播放失败可抛异常。 |
| `listen_speak.say_event(event,offline=False,**params)` | 模板名 `start/arrive` 传 `number`，`fail` 传 `reason`，`complete/empty/offline` 无需参数。对有效模板，播放成功返回 `True`，失败返回 `False`。`listen_speak.tts.prewarm()` 预生成固定提示音。 |

独立入口有 `python -m listen_speak.asr`、`python -m listen_speak.llm "口令"`、`python -m listen_speak.tts [--prewarm]`。需要完整识别时在已配置进程调用 `listen()`。音频设备、Vosk/Piper 模型与密钥的配置见[语音模块](listen_speak/README.md)。
