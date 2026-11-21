"""2 m 场地，原点在图纸左下，+X 向右，+Y 向上。内部长度均为毫米。"""

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from .board import BOARD_LAYOUT, Plane, fit_plane, square


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
    wall_tags: Optional[list] = None  # 显式实测布局，覆盖旧版四墙中心/高度表
    docks_m: dict = field(default_factory=lambda: dict(DOCKS_M))
    bounds_m: tuple = (0., 0., 2., 2.)  # 允许车心目标的 xmin,ymin,xmax,ymax

    @classmethod
    def load(cls, path=None):
        return cls(**json.loads(Path(path).read_text(encoding="utf-8"))) if path else cls()

    def wall_ids(self):
        return set(WALLS) if self.wall_tags is None else {tag["id"] for tag in self.wall_tags}

    def wall_layout(self):
        """每个直立标签的中心、从码面看向右的单位向量、黑框边长，长度用毫米。"""
        if self.wall_tags is None:
            return {i: (np.array([*center, self.wall_heights_mm[str(i)]]), right, 200.)
                    for i, (center, right) in WALLS.items() if str(i) in self.wall_heights_mm}
        layout = {}
        for tag in self.wall_tags:
            ident = tag["id"]
            center = np.asarray(tag["center_mm"], dtype=float)
            angle = math.radians(tag["right_yaw_deg"])
            side = tag["side_mm"]
            if (type(ident) is not int or ident < 0 or ident in layout or center.shape != (3,)
                    or not np.isfinite(center).all() or center[2] <= 0 or not math.isfinite(angle)
                    or not math.isfinite(side) or side <= 0):
                raise ValueError(f"墙标 {ident} 的 ID、中心、方向或尺寸无效，请填写实测值")
            layout[ident] = center, (math.cos(angle), math.sin(angle), 0.), side
        return layout

    def floor_layout(self):
        a = math.radians(self.board_yaw_deg)
        rotation = np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])
        layout = {k: xy @ rotation + self.board_origin_mm for k, xy in BOARD_LAYOUT.items()}
        for tag in self.floor_tags:
            key = tag["family"], tag["id"]
            if key in layout or (key[0] == self.wall_family and key[1] in self.wall_ids()):
                raise ValueError(f"附加地标与棋盘或围挡重复 {key}")
            layout[key] = square(*tag["center_mm"], tag["side_mm"], tag.get("yaw_deg", 0))
        return layout


def wall_views(tags, K, layout, family):
    """生成墙标平面 PnP 的正深度、正面候选。输入像素已去畸变。"""
    up = np.array([0., 0., 1.])
    for tag_id, (origin, right, side) in layout.items():
        key = family, tag_id
        if key not in tags:
            continue
        local = np.column_stack([square(0, 0, side), np.zeros(4)])
        normal = np.cross(right, up)
        basis = np.column_stack([right, up, normal])
        uv = np.asarray(tags[key], dtype=float)
        _, rotations, translations, _ = cv2.solvePnPGeneric(local, uv, K, None, flags=cv2.SOLVEPNP_IPPE)
        for rvec, tvec in zip(rotations, translations):
            rotation = cv2.Rodrigues(rvec)[0] @ basis.T
            t = tvec.ravel() - rotation @ origin
            camera = -rotation.T @ t
            if not np.isfinite(camera).all() or (camera - origin) @ normal <= 0:
                continue
            depth = (local @ cv2.Rodrigues(rvec)[0].T + tvec.ravel())[:, 2]
            if np.any(depth <= 0):
                continue
            pixels = cv2.projectPoints(local, rvec, tvec, K, None)[0].reshape(-1, 2)
            rms = float(np.sqrt(np.mean(np.sum((pixels - uv)**2, axis=1))))
            if math.isfinite(rms):
                yield key, rotation, t, camera, rms


def wall_plane(tags, mount, field):
    """下视相机用高度与俯视方向消除墙标平面解的歧义。"""
    if not mount.camera_matrix:
        return None
    K = np.asarray(mount.camera_matrix, dtype=float)
    best, best_score = None, float("inf")
    for key, rotation, t, camera, rms in wall_views(tags, K, field.wall_layout(), field.wall_family):
        height_error = abs(camera[2] - mount.height_mm)
        score = rms + height_error / max(1., field.height_tolerance_mm)
        if (height_error <= field.height_tolerance_mm and rotation[2, 2] <= -0.65
                and rms < 3 and score < best_score):
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
    extra = {k: xy for k, xy in layout.items() if k not in BOARD_LAYOUT}
    plane = fit_plane(tags, extra, min_tags=1)
    if plane is not None:
        return plane, "floor"
    plane = wall_plane(tags, mount, field)
    return plane, "wall" if plane is not None else None
