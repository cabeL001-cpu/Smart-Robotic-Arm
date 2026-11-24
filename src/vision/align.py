"""单相机闭环到格。安装标定可用图片，实际控制使用实时相机。"""

import argparse
from dataclasses import replace
from math import cos, radians, sin

import numpy as np

from .arm import Hold, measure, target_xy
from .board import axes, fit_plane, project
from .camera import Mount, frames, live_source
from .tags import Detector

if "." in __package__:
    from ..arm import ArmConfig, set_joints
    from ..uno import open_uno, DEFAULT_PORT
else:
    from arm import ArmConfig, set_joints
    from uno import open_uno, DEFAULT_PORT


def observations(source, mount, cell):
    detector = Detector(("36h11", mount.tip_family))
    for t, frame in frames(source, mount):
        tags = detector.detect(frame)
        plane = fit_plane(tags)
        yield t, plane, measure(tags, plane, frame.shape[1::-1], cell, mount)


def align(cell, samples, send, mount, config, settle_s=0.6, max_steps=10, timeout_s=20):
    """samples 提供时间、棋盘变换和末端测量，send 只接收两个关节角。"""
    if not mount.arm_base_px or not mount.arm_x_px:
        raise ValueError("先标定随车相机下的基座与机械臂 +X 方向")
    hold = Hold()
    target = target_xy(cell)
    aim, previous = target.copy(), (0., 0.)
    first_t, sent_at, steps = None, float("-inf"), 0
    for t, plane, obs in samples:
        if first_t is None:
            first_t = t
        if t - first_t > timeout_s:
            break
        if hold.update(t, obs):
            return obs
        if obs is None or obs.error_mm < 6.7 or t - sent_at < settle_s:
            continue
        if steps >= max_steps:
            raise RuntimeError("已达到修正次数，末端仍未稳定到位")
        if steps:
            # 从实测残差修正下一次笛卡尔指令，给舵机留出响应时间。
            aim += 0.7 * (target - np.asarray(obs.tip_mm))
        base, yaw = axes(plane, mount.arm_base_px, mount.arm_x_px)
        geometry = replace(config.geometry, base_x_mm=base[0], base_y_mm=base[1], base_yaw_deg=yaw)
        previous = config.solve(*aim, geometry=geometry, previous=previous)
        send(previous)
        sent_at, steps = t, steps + 1
    raise RuntimeError("观测结束或超时，未取得连续两秒到位观测")


def main():
    p = argparse.ArgumentParser(description="标定基座投影或实时闭环对格")
    p.add_argument("--source", default="0")
    p.add_argument("--config", help="相机参数 JSON")
    p.add_argument("--arm-config", help="舵机标定 JSON")
    p.add_argument("--port", default=DEFAULT_PORT)
    action = p.add_mutually_exclusive_group(required=True)
    action.add_argument("--cell", type=int, choices=range(9))
    action.add_argument("--save-mount", metavar="JSON", help="从已知基座位置标定两个投影像素")
    p.add_argument("--base", type=float, nargs=3, default=(-150, 100, 0), metavar=("X毫米", "Y毫米", "方向度"))
    args = p.parse_args()
    mount = Mount.load(args.config)
    if args.save_mount:
        detector = Detector(("36h11",))
        for _, frame in frames(args.source, mount):
            plane = fit_plane(detector.detect(frame))
            if plane is None:
                continue
            x, y, yaw = args.base
            a = radians(yaw)
            points = project(plane.to_pixel, [(x, y), (x + 100*cos(a), y + 100*sin(a))])
            mount.arm_base_px, mount.arm_x_px = points.tolist()
            mount.image_size = frame.shape[1::-1]
            mount.save(args.save_mount)
            return
        p.exit(1, "标定图像中没有足够的已知棋盘角标。\n")
    if not live_source(args.source):
        p.error("驱动舵机需要实时相机编号。录像可用 vision.arm 测量")
    config = ArmConfig.load(args.arm_config) if args.arm_config else ArmConfig()
    if not mount.arm_base_px or not mount.arm_x_px:
        p.error("请先用 --save-mount 生成基座投影标定")
    try:
        config.joint_limits()
    except ValueError as error:
        p.error(str(error))
    with open_uno(args.port) as link:
        result = align(args.cell, observations(args.source, mount, args.cell),
                       lambda angles: set_joints(angles, config=config, link=link), mount, config)
        print(f"连续到位两秒，末帧误差 {result.error_mm:.2f} mm")


if __name__ == "__main__":
    main()
