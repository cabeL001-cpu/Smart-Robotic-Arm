"""前视相机用竖直墙标测量地面上的车心与航向，独立运行时只读图像。"""

import argparse
from dataclasses import asdict, dataclass
import json
import math

import cv2
import numpy as np

from .camera import Mount, frames
from .field import Field, wall_views
from .tags import Detector


@dataclass(frozen=True)
class FrontFix:
    x_m: float
    y_m: float
    yaw_rad: float
    tag_ids: tuple
    rms_px: float


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class FrontLocalizer:
    def __init__(self, mount, field, *, max_rms_px=2.5, max_ground_mm=80., max_tilt_deg=12.):
        self.K = np.asarray(mount.camera_matrix, dtype=float)
        self.offset = np.asarray(mount.front_xyz_mm, dtype=float)
        if (self.K.shape != (3, 3) or not np.isfinite(self.K).all()
                or min(self.K[0, 0], self.K[1, 1]) <= 0 or not np.allclose(self.K[2], [0, 0, 1])
                or len(mount.image_size) != 2 or min(mount.image_size) <= 0):
            raise ValueError("前摄配置需填写当前分辨率 image_size 和实测 camera_matrix")
        if (self.offset.shape != (3,) or not np.isfinite(self.offset).all() or self.offset[2] <= 0
                or len(mount.front_ypr_deg) != 3 or not all(map(math.isfinite, mount.front_ypr_deg))):
            raise ValueError("前摄配置需填写光心 front_xyz_mm 和安装角 front_ypr_deg")
        if not all(math.isfinite(v) and v > 0 for v in (max_rms_px, max_ground_mm, max_tilt_deg)):
            raise ValueError("前摄重投影、车心高度和倾斜误差门限必须为正数")
        self.rms_limit, self.height_limit = max_rms_px, max_ground_mm
        self.tilt_limit = math.radians(max_tilt_deg)
        yaw, pitch, roll = map(math.radians, mount.front_ypr_deg)
        rz = cv2.Rodrigues(np.array([0., 0., yaw]))[0]
        ry = cv2.Rodrigues(np.array([0., -pitch, 0.]))[0]
        rx = cv2.Rodrigues(np.array([roll, 0., 0.]))[0]
        # 相机轴为右、下、前，车体轴为前、左、上。正仰角表示镜头向上。
        neutral = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
        self.camera_to_body = rz @ ry @ rx @ neutral
        self.layout = field.wall_layout()
        if not self.layout:
            raise ValueError("缺少已测量的墙标中心、高度与黑框尺寸")
        self.family = field.wall_family
        self.detector = Detector((self.family,), board=False)

    def measure(self, frame):
        """对已按 mount.rectify 去畸变的图像估计车心。歧义或无有效墙标时返回 None。"""
        tags = self.detector.detect(frame)
        candidates = {}
        for key, rotation, _, camera, rms in wall_views(tags, self.K, self.layout, self.family):
            body_to_world = rotation.T @ self.camera_to_body.T
            center = camera - body_to_world @ self.offset
            tilt = math.acos(float(np.clip(body_to_world[2, 2], -1., 1.)))
            if rms > self.rms_limit or abs(center[2]) > self.height_limit or tilt > self.tilt_limit:
                continue
            yaw = math.atan2(body_to_world[1, 0], body_to_world[0, 0])
            score = rms / self.rms_limit + abs(center[2]) / self.height_limit + tilt / self.tilt_limit
            pose = FrontFix(float(center[0]) / 1000, float(center[1]) / 1000, yaw, (key[1],), rms)
            candidates.setdefault(key[1], []).append((score, pose))

        poses = []
        for choices in candidates.values():
            choices.sort(key=lambda item: item[0])
            score, best = choices[0]
            # 一块平面标可能有两个几乎同样好的镜像解。相差明显时等待另一视角。
            if any(cost - score < .20 and not self._agree(best, other) for cost, other in choices[1:]):
                continue
            poses.append(best)
        if not poses or any(not self._agree(a, b) for i, a in enumerate(poses) for b in poses[i+1:]):
            return None
        return FrontFix(sum(p.x_m for p in poses) / len(poses), sum(p.y_m for p in poses) / len(poses),
                        math.atan2(sum(math.sin(p.yaw_rad) for p in poses),
                                   sum(math.cos(p.yaw_rad) for p in poses)),
                        tuple(sorted(p.tag_ids[0] for p in poses)), max(p.rms_px for p in poses))

    @staticmethod
    def _agree(a, b):
        return (math.hypot(a.x_m - b.x_m, a.y_m - b.y_m) <= .12
                and abs(angle_difference(a.yaw_rad, b.yaw_rad)) <= math.radians(12))


def main():
    parser = argparse.ArgumentParser(description="前摄墙标米制车心定位（只读）")
    parser.add_argument("--source", required=True, help="前摄设备、图片或录像")
    parser.add_argument("--config", required=True, help="前摄内参和安装外参 JSON")
    parser.add_argument("--field", required=True, help="实测墙标地图 JSON")
    parser.add_argument("--frames", type=int, help="最多输出的帧数")
    args = parser.parse_args()
    if args.frames is not None and args.frames < 1:
        parser.error("--frames 必须为正整数")
    mount = Mount.load(args.config)
    estimator = FrontLocalizer(mount, Field.load(args.field))
    for n, (t, frame) in enumerate(frames(args.source, mount), 1):
        fix = estimator.measure(frame)
        print(json.dumps({"t": t, "pose": asdict(fix) if fix else None}, allow_nan=False), flush=True)
        if n == args.frames:
            break


if __name__ == "__main__":
    main()
