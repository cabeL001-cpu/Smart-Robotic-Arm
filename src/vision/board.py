"""利用已知共面标签建立像素与毫米之间的单应变换。棋盘坐标原点在左下角。"""

from dataclasses import dataclass
import math

import cv2
import numpy as np


def square(x, y, side, yaw_deg=0):
    half = side / 2
    local = np.array([[-half, half], [half, half], [half, -half], [-half, -half]])
    a = math.radians(yaw_deg)
    return local @ np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]]) + (x, y)


BOARD = {("36h11", i): square(x, y, 20) for i, (x, y) in
         enumerate(((10, 10), (190, 10), (10, 190), (190, 190)))}


# 实物印刷版的黑框约 15 mm，外侧两条边贴着 200 mm 棋盘的外沿。
BOARD_LAYOUT = dict(BOARD)
for ident, (x, y) in enumerate(((7.5, 7.5), (192.5, 7.5), (7.5, 192.5), (192.5, 192.5))):
    BOARD_LAYOUT["printed5", ident] = square(x, y, 15)


def project(matrix, points):
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, matrix).reshape(-1, 2)


@dataclass
class Plane:
    to_pixel: np.ndarray
    to_world: np.ndarray
    tags_used: tuple
    rms_px: float


def fit_plane(tags, layout=BOARD_LAYOUT, min_tags=2, tolerance_px=3.0):
    keys = sorted(layout.keys() & tags.keys())
    if any(key[0] == "printed5" for key in keys):
        # 实物板与旧模板二选一，排除与旧角标同号的墙标。
        keys = [key for key in keys if key not in BOARD]
    if len(keys) < min_tags:
        return None
    xy = np.vstack([layout[k] for k in keys])
    uv = np.vstack([tags[k] for k in keys])
    h, mask = cv2.findHomography(xy, uv, cv2.RANSAC, tolerance_px)
    if h is None:
        return None
    # 每个计入的标记至少有三个一致角点，随后用这些内点重新拟合。
    mask = mask.ravel().astype(bool).reshape(-1, 4)
    valid_tags = mask.sum(axis=1) >= 3
    if valid_tags.sum() < min_tags:
        return None
    keep = (mask & valid_tags[:, None]).ravel()
    h, _ = cv2.findHomography(xy[keep], uv[keep], 0)
    if h is None or np.linalg.matrix_rank(h) < 3:
        return None
    residual = project(h, xy[keep]) - uv[keep]
    rms = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
    if rms > tolerance_px:
        return None
    return Plane(h, np.linalg.inv(h), tuple(k for k, ok in zip(keys, valid_tags) if ok), rms)


def axes(plane, origin_px, x_px):
    """把刚性安装的两个地面投影像素变换成平面位置和朝向。"""
    origin, ahead = project(plane.to_world, [origin_px, x_px])
    dx, dy = ahead - origin
    if math.hypot(dx, dy) < 1e-6:
        raise ValueError("两个方向标定点重合")
    return origin, math.degrees(math.atan2(dy, dx))
