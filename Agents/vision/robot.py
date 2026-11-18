"""从随车下视图像解算车体中心和朝向，同时可输出同一帧中的末端观测。"""

import argparse
from dataclasses import dataclass, asdict
import json
import math

from .arm import measure
from .board import axes, fit_plane, project
from .camera import Mount, frames
from .field import Field, locate_floor
from .tags import Detector


@dataclass(frozen=True)
class Pose:
    x_m: float
    y_m: float
    yaw_rad: float
    source: str
    tags_used: int
    rms_px: float


def calibrate(mount, plane, size, x_m, y_m, yaw_deg):
    """用车体处于已知位姿的一帧，标定车心和车前方 100 mm 的地面投影像素。"""
    x, y, a = x_m * 1000, y_m * 1000, math.radians(yaw_deg)
    pixels = project(plane.to_pixel, [(x, y), (x + 100*math.cos(a), y + 100*math.sin(a))])
    mount.robot_center_px, mount.robot_forward_px = pixels.tolist()
    mount.image_size = size


def pose_from_plane(plane, source, mount):
    if plane is None:
        return None
    if not mount.robot_center_px or not mount.robot_forward_px:
        raise ValueError("先用已知的车心位置与朝向标定相机安装参数")
    center, yaw = axes(plane, mount.robot_center_px, mount.robot_forward_px)
    return Pose(float(center[0] / 1000), float(center[1] / 1000), math.radians(yaw),
                source, len(plane.tags_used), plane.rms_px)


def main():
    p = argparse.ArgumentParser(description="单相机机器人场地定位")
    p.add_argument("--source", default="0")
    p.add_argument("--config", help="随车相机标定 JSON")
    p.add_argument("--field", help="固定场地标签布局 JSON")
    p.add_argument("--calibrate", nargs=3, type=float, metavar=("X米", "Y米", "朝向度"))
    p.add_argument("--save-mount", help="写入标定后的 JSON")
    p.add_argument("--cell", type=int, choices=range(9), help="同时测量同一帧的机械臂格心误差")
    args = p.parse_args()
    if bool(args.calibrate) != bool(args.save_mount):
        p.error("--calibrate 与 --save-mount 配套使用")
    mount, field = Mount.load(args.config), Field.load(args.field)
    if not args.calibrate and not (mount.robot_center_px and mount.robot_forward_px):
        p.error("请先 --calibrate，再用 --config 指定标定文件")
    families = ["36h11", field.wall_family, *(item["family"] for item in field.floor_tags)]
    if args.cell is not None:
        families.append(mount.tip_family)
    detector = Detector(families)
    for t, frame in frames(args.source, mount):
        size = frame.shape[1::-1]
        tags = detector.detect(frame)
        plane, source = locate_floor(tags, mount, field)
        if args.calibrate:
            if plane is None:
                continue
            calibrate(mount, plane, size, *args.calibrate)
            mount.save(args.save_mount)
            return
        pose = pose_from_plane(plane, source, mount)
        record = {"t": t, "pose": asdict(pose) if pose else None}
        if args.cell is not None:
            obs = measure(tags, fit_plane(tags), size, args.cell, mount)
            record["arm"] = asdict(obs) if obs else None
        print(json.dumps(record, allow_nan=False), flush=True)
    if args.calibrate:
        p.exit(1, "当前图片或录像中没有可用于标定的场地参照。\n")


if __name__ == "__main__":
    main()
