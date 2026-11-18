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
python3 -m vision.align --source 0 --config rig.json --arm-config arm-calibration.json --cell 4 --port /dev/ttyACM0
~~~

首次用实测基座姿态求逆解，待舵机响应后按实测偏差修正。棋盘与末端同时可见时才更新动作，连续两秒到位后返回，超时或达到修正次数会报出未到位。测试时先单独运行 `vision.arm` 核对尺度、方向和观测，再接通机械臂。[关节和串口标定](../arm/README.md)
