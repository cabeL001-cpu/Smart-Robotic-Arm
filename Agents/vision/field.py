"""2 m 场地，原点在图纸左下，+X 向右，+Y 向上。内部长度均为毫米。"""

from dataclasses import dataclass, field
import json
import math
from pathlib import Path

import cv2
import numpy as np

from .board import BOARD, Plane, fit_plane, square


DOCKS_M = {"A": (0.5, 1.5), "B": (1.5, 1.5), "C": (1.5, 0.5), "D": (0.5, 0.5)}
# 按打印码正面看，列出墙标中心的 XY 和它的水平向右轴，标签顶边朝上。
WALLS = {1: ((1000, 2000), (1, 0, 0)), 2: ((1000, 0), (-1, 0, 0)),
         3: ((0, 1000), (0, 1, 0)), 4: ((2000, 1000), (0, -1, 0))}


@dataclass
class Field:
    board_origin_mm: tuple = (900., 900.)
    board_yaw_deg: float = 0.
    floor_tags: list = field(default_factory=list)
    wall_heights_mm: dict = field(default_factory=dict)
    wall_family: str = "36h11"
    height_tolerance_mm: float = 250.

    @classmethod
    def load(cls, path=None):
        return cls(**json.loads(Path(path).read_text(encoding="utf-8"))) if path else cls()

    def floor_layout(self):
        a = math.radians(self.board_yaw_deg)
        rotation = np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])
        layout = {k: xy @ rotation + self.board_origin_mm for k, xy in BOARD.items()}
        for tag in self.floor_tags:
            key = tag["family"], tag["id"]
            if key in layout or (key[0] == self.wall_family and key[1] in WALLS):
                raise ValueError(f"附加地标与棋盘或围挡重复 {key}")
            layout[key] = square(*tag["center_mm"], tag["side_mm"], tag.get("yaw_deg", 0))
        return layout


def wall_plane(tags, mount, field):
    """用可见的直立墙标解相机位姿，再构造地面单应。没有实测内参或高度时跳过。"""
    if not mount.camera_matrix:
        return None
    K = np.asarray(mount.camera_matrix, dtype=float)
    local = np.column_stack([square(0, 0, 200), np.zeros(4)])
    up = np.array([0., 0., 1.])
    best, best_score = None, float("inf")
    for tag_id, (center_xy, right) in WALLS.items():
        key = field.wall_family, tag_id
        height = field.wall_heights_mm.get(str(tag_id))
        if key not in tags or height is None:
            continue
        origin = np.array([*center_xy, height])
        normal = np.cross(right, up)
        basis = np.column_stack([right, up, normal])
        uv = np.asarray(tags[key], dtype=float)
        _, rotations, translations, _ = cv2.solvePnPGeneric(local, uv, K, None, flags=cv2.SOLVEPNP_IPPE)
        for rvec, tvec in zip(rotations, translations):
            rotation = cv2.Rodrigues(rvec)[0] @ basis.T
            t = tvec.ravel() - rotation @ origin
            camera = -rotation.T @ t
            height_error = abs(camera[2] - mount.height_mm)
            if (height_error > field.height_tolerance_mm or rotation[2, 2] > -0.65
                    or (camera - origin) @ normal <= 0):
                continue
            depth = (local @ cv2.Rodrigues(rvec)[0].T + tvec.ravel())[:, 2]
            if np.any(depth <= 0):
                continue
            pixels = cv2.projectPoints(local, rvec, tvec, K, None)[0].reshape(-1, 2)
            rms = float(np.sqrt(np.mean(np.sum((pixels - uv)**2, axis=1))))
            score = rms + height_error / max(1., field.height_tolerance_mm)
            if rms < 3 and score < best_score:
                h = K @ np.column_stack([rotation[:, 0], rotation[:, 1], t])
                best = Plane(h, np.linalg.inv(h), (key,), rms)
                best_score = score
    return best


def locate_floor(tags, mount, field):
    layout = field.floor_layout()
    plane = fit_plane(tags, layout)
    if plane is not None:
        return plane, "floor"
    # 独有编码的附加平面地标可单独解算。棋盘小标签仍要求多个标签互证。
    extra = {k: xy for k, xy in layout.items() if k not in BOARD}
    plane = fit_plane(tags, extra, min_tags=1)
    if plane is not None:
        return plane, "floor"
    plane = wall_plane(tags, mount, field)
    return plane, "wall" if plane is not None else None
