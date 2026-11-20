# 随车单相机视觉

相机固定在车上，向下看，离地约 1.5 m。棋盘固定于场地时，它既提供毫米尺，也提供车外的绝对参照。相机移动后，每一帧重新估计棋盘平面。

## 机械臂末端与格心

从 `Agents` 目录执行。相机、图片和录像走同一套检测与测量代码。

~~~bash
python3 -m pip install -r vision/requirements.txt
python3 -m unittest vision.test_arm
python3 -m vision.arm --source 0 --cell 4 --config mount.json
python3 -m vision.arm --source board.jpg --cell 4
~~~

输出 JSON 给出末端、目标、欧氏误差（毫米）、参与拟合的标签数和重投影误差（像素）。`measurement: null` 表示该帧缺少参照或末端。连续新帧误差小于 6.7 mm 达 2 秒才出现 `arrived: true`。图片适合检查单帧测量，录像可检查连续到位。

棋盘为 200 × 200 mm，左下为 (0, 0)，左下、右下、左上、右上分别为 tag36h11 的 0、1、2、3。每个黑边外沿为 20 mm，印刷方向按模板。程序用至少两个标签的角点拟合平面。末端默认使用实物照片中的 **tag36h10 / 100**，家族和 ID 均可改。只看到末端时会报告观测缺失。[棋盘模板](../image-7.png) · [OpenCV 标签角点约定](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)

## 相机安装标定

配置文件可只写需要覆盖的字段。以下为参数格式，按实物替换数值。

~~~json
{
  "image_size": [1280, 720],
  "height_mm": 1500,
  "tip_height_mm": 0,
  "tip_offset_mm": [0, 0],
  "tip_family": "36h10",
  "tip_id": 100
}
~~~

`height_mm` 从镜头中心量到棋盘平面，`tip_height_mm` 从棋盘量到末端标签平面。两者用于修正不同高度的视差。`tip_offset_mm` 是从末端标签中心到指尖在标签右、上方向的偏移。倾斜安装时填写 `nadir_px`，即镜头铅垂线在棋盘平面的落点像素。省略时按垂直安装、落点在图像中心处理。有内参时填写 OpenCV 的 `camera_matrix`（3×3）和 `distortion`，所有像素标定均在同分辨率的去畸变图像上进行。验收精度要由实车标尺测量确认。

## 基座标定与实时闭环

将车停在已知对齐工位。从棋盘左下角量肩轴的平面坐标，再量零肩角方向相对棋盘 +X 的逆时针角度。图纸名义值为 (−150, 100) mm、0°。下列第一条命令从画面反算肩轴和朝前点的像素，写入供随车相机复用的安装参数。此后车辆换位置时，两点会在当前帧转换回新的棋盘坐标。相机与基座的相对安装须固定。

~~~bash
python3 -m vision.align --source 0 --config mount.json --base -150 100 0 --save-mount rig.json
python3 -m vision.align --source 0 --config rig.json --arm-config arm-calibration.json --cell 4 --port "$UNO_PORT"
~~~

`UNO_PORT` 取蓝牙链路提供的串行端点。先按实车标定肩、肘的最大位置、退回方向和脉宽，烧录相同限位。闭环对每次修正都检查 90° 行程和端点余量。超出可达区域时报告错误。首次用实测基座姿态求逆解，待舵机响应后按实测偏差修正。棋盘与末端同时可见时才更新动作，连续两秒到位后返回，超时或达到修正次数会报出未到位。测试时先单独运行 `vision.arm` 核对尺度、方向和观测，再接通机械臂。[关节和串口标定](../arm/README.md)

## 机器人场地位姿

场地左下角为原点，右/上为 +X/+Y，朝 +X 为 0，逆时针为正。按场地图，棋盘左下在 (0.9, 0.9) m，棋盘上边朝场地上方。A、B、C、D 分别为 (0.5, 1.5)、(1.5, 1.5)、(1.5, 0.5)、(0.5, 0.5) m。棋盘须固定在场地中。[场地布局](../image-2.png)

先将车心停在一个已知位置，量车头朝向。下面的数值示例表示车心在 (1, 1) m，车头指向 +X。程序用这一帧标定车心与车前方的地面投影像素。保持相机安装、分辨率与底盘离地姿态一致，之后用当前帧即可恢复车体位姿。`--config rig.json` 可沿用机械臂安装参数。

~~~bash
python3 -m unittest vision.test_robot
python3 -m vision.robot --source 0 --config rig.json --calibrate 1 1 0 --save-mount vehicle.json
python3 -m vision.robot --source 0 --config vehicle.json --cell 4
~~~

每行 JSON 的 `pose` 包含 `x_m`、`y_m`、`yaw_rad`、参照来源和重投影误差。`--cell` 让两个视觉模块使用同一次曝光和标签检测。相机视野随车移动，失去固定参照时该帧输出 `pose: null`。单相机无法从无标志地面直接恢复绝对位置。

场地调整后可通过 `--field field.json` 指定棋盘摆放位置及可选地标。下例按毫米填写，墙标高度仅作文件格式示例，使用前实测各黑色方框中心距地面的高度。

~~~json
{
  "board_origin_mm": [900, 900],
  "board_yaw_deg": 0,
  "floor_tags": [],
  "wall_family": "36h11",
  "wall_heights_mm": {"1": 180, "2": 180, "3": 180, "4": 180}
}
~~~

默认使用棋盘。墙标只有在实际进入画面、配置镜头内参和实测中心高度时参与独立的三维位姿解算。按图使用 200 mm 正方形，码面朝场内、顶边向上，物理解还须与下视方向和相机高度一致。可选的水平地标记录格式为 `{"family":"36h11","id":20,"center_mm":[500,500],"side_mm":100,"yaw_deg":0}`，编码需区别于棋盘和墙标。

实时位姿也可通过 JSON 行供 IMU 校正。两个进程共用同一台树莓派的单调时钟。[视觉与 IMU 联合运行](../move/imu/README.md)
