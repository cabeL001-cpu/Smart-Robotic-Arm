"""机械臂视觉测量与两秒到位判定。仅使用单帧检测结果，不访问舵机。"""

import argparse
from dataclasses import dataclass, asdict
import json
import math

import numpy as np

from .board import fit_plane, project
from .camera import Mount, frames
from .tags import Detector


def target_xy(cell):
    if cell not in range(9):
        raise ValueError("格号应为 0 至 8")
    row, col = divmod(cell, 3)
    return np.array([(col + 0.5) * 200 / 3, (row + 0.5) * 200 / 3])


@dataclass(frozen=True)
class Measurement:
    tip_mm: tuple
    target_mm: tuple
    error_mm: float
    tags_used: int
    rms_px: float


def measure(tags, plane, size, cell, mount):
    key = mount.tip_family, mount.tip_id
    if plane is None or key not in tags:
        return None
    if not 0 <= mount.tip_height_mm < mount.height_mm:
        raise ValueError("末端标签高度须处于棋盘与镜头之间")
    # 投影射线先与棋盘相交，再按实测高度插值到与棋盘平行的末端平面。
    foot_px = mount.nadir_px or ((size[0] - 1) / 2, (size[1] - 1) / 2)
    foot = project(plane.to_world, [foot_px])[0]
    corners = project(plane.to_world, tags[key])
    corners += (foot - corners) * (mount.tip_height_mm / mount.height_mm)
    right = (corners[1] + corners[2]) - (corners[0] + corners[3])
    up = (corners[0] + corners[1]) - (corners[2] + corners[3])
    tip = (corners.mean(axis=0) + mount.tip_offset_mm[0] * right / np.linalg.norm(right)
           + mount.tip_offset_mm[1] * up / np.linalg.norm(up))
    target = target_xy(cell)
    return Measurement(tuple(tip), tuple(target), math.dist(tip, target), len(plane.tags_used), plane.rms_px)


class Hold:
    """只累计连续可见、误差小于 6.7 mm 的新帧。"""
    def __init__(self, seconds=2.0, max_gap=0.5):
        self.seconds, self.max_gap = seconds, max_gap
        self.start = self.last = None

    def update(self, t, observation):
        if self.last is not None and (t <= self.last or t - self.last > self.max_gap):
            self.start = None
        self.last = t
        if observation is None or observation.error_mm >= 6.7:
            self.start = None
            return False
        if self.start is None:
            self.start = t
        return t - self.start >= self.seconds


def main():
    parser = argparse.ArgumentParser(description="单相机测量末端与格心的毫米误差")
    parser.add_argument("--source", default="0", help="相机编号、图片或录像")
    parser.add_argument("--cell", type=int, choices=range(9), required=True)
    parser.add_argument("--config", help="相机安装参数 JSON")
    args = parser.parse_args()
    mount = Mount.load(args.config)
    detector, hold = Detector(("36h11", mount.tip_family)), Hold()
    for t, frame in frames(args.source, mount):
        tags = detector.detect(frame)
        obs = measure(tags, fit_plane(tags), frame.shape[1::-1], args.cell, mount)
        print(json.dumps({"t": t, "measurement": asdict(obs) if obs else None,
                          "arrived": hold.update(t, obs)}, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
