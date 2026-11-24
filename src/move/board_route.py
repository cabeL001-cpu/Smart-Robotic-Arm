"""九宫格周围的车心路径，坐标和距离均使用米。"""

from dataclasses import dataclass
from heapq import heappop, heappush
import math
from typing import Optional

from .imu import rotate


@dataclass(frozen=True)
class BoardApproach:
    plate_side_m: float = .30
    chassis_length_m: float = .34
    chassis_width_m: float = .34
    clearance_m: float = .05
    staging_distance_m: float = .50
    work_distance_m: Optional[float] = None
    heading_deg: float = 90.
    heading_tolerance_deg: float = 4.

    def __post_init__(self):
        distances = (self.plate_side_m, self.chassis_length_m, self.chassis_width_m,
                     self.clearance_m, self.staging_distance_m)
        if not all(math.isfinite(v) and v > 0 for v in distances):
            raise ValueError("板尺寸、底盘尺寸、净空和预靠距离须为有限正数")
        if self.work_distance_m is not None and not 0 < self.work_distance_m < self.staging_distance_m:
            raise ValueError("作业距离须大于零且小于预靠距离")
        if (not math.isfinite(self.heading_deg) or self.heading_deg % 90 != 0
                or not 0 < self.heading_tolerance_deg <= 10):
            raise ValueError("车头取场地轴向，航向容差须在 0 至 10 度之间")


@dataclass(frozen=True)
class TravelArea:
    bounds: tuple
    obstacle: tuple

    def contains(self, point):
        x, y = point
        x0, y0, x1, y1 = self.bounds
        a, b, c, d = self.obstacle
        return x0 <= x <= x1 and y0 <= y <= y1 and not (a <= x <= c and b <= y <= d)

    def allows(self, start, end):
        if not self.contains(start) or not self.contains(end):
            return False
        # 求线段与扩大后矩形在两轴上的相交参数区间。擦边也视为相交。
        first, last = 0., 1.
        for p, q, low, high in zip(start, end, self.obstacle[:2], self.obstacle[2:]):
            if p == q:
                if not low <= p <= high:
                    return True
                continue
            near, far = sorted(((low - p) / (q - p), (high - p) / (q - p)))
            first, last = max(first, near), min(last, far)
            if first > last:
                return True
        return False

    def clip(self, start, direction, distance, target):
        def clear(amount):
            point = tuple(p + amount * v for p, v in zip(start, direction))
            return self.allows(start, point) and self.allows(point, target)

        if clear(distance):
            return distance
        low, high = 0., distance
        for _ in range(16):
            mid = (low + high) / 2
            if clear(mid):
                low = mid
            else:
                high = mid
        return low

    def path(self, start, goal, waypoint_margin):
        if not self.contains(start) or not self.contains(goal):
            raise ValueError("起点或目标离板边或围挡过近，车心路径无足够净空")
        if self.allows(start, goal):
            return [goal]
        x0, y0, x1, y1 = self.obstacle
        corners = [(x, y) for x in (x0 - waypoint_margin, x1 + waypoint_margin)
                   for y in (y0 - waypoint_margin, y1 + waypoint_margin)]
        points = [start, goal] + [p for p in corners if self.contains(p)]
        queue = [(0., [0])]
        visited = set()
        while queue:
            length, route = heappop(queue)
            node = route[-1]
            if node == 1:
                return [points[i] for i in route[1:]]
            if node in visited:
                continue
            visited.add(node)
            for i, point in enumerate(points):
                if i not in visited and self.allows(points[node], point):
                    heappush(queue, (length + math.dist(points[node], point), route + [i]))
        raise ValueError("亚克力板和围挡之间没有可通行路线")


class BoardRoute:
    def __init__(self, settings, bounds, board_origin_mm, board_yaw_deg):
        self.settings = settings
        self.heading = math.radians(settings.heading_deg % 360)
        yaw = math.radians(board_yaw_deg)
        # 地图记录的是 20 cm 格线的左下角。按实物板与格线同心、同向建模。
        offset = rotate(.10, .10, yaw)
        x, y = (value / 1000 + delta for value, delta in zip(board_origin_mm, offset))
        plate_extent = settings.plate_side_m / 2 * (abs(math.cos(yaw)) + abs(math.sin(yaw)))
        self.staging = (x, y - settings.staging_distance_m)
        self.work = (None if settings.work_distance_m is None
                     else (x, y - settings.work_distance_m))
        half_l, half_w = settings.chassis_length_m / 2, settings.chassis_width_m / 2
        radius = math.hypot(half_l, half_w)
        yaw_margin = math.sin(math.radians(settings.heading_tolerance_deg))
        # 用轴向车身的投影加上允许航向偏差，供最后一段固定航向的平移使用。
        if settings.heading_deg % 180 == 90:
            half_l, half_w = half_w, half_l
        near_x, near_y = half_l + half_w * yaw_margin, half_w + half_l * yaw_margin

        def area(rx, ry):
            rx, ry = rx + settings.clearance_m, ry + settings.clearance_m
            x0, y0, x1, y1 = bounds
            return TravelArea((x0 + rx, y0 + ry, x1 - rx, y1 - ry),
                              (x - plate_extent - rx, y - plate_extent - ry,
                               x + plate_extent + rx, y + plate_extent + ry))

        self.travel = area(radius, radius)
        self.near = area(near_x, near_y)
        if not self.travel.contains(self.staging):
            raise ValueError("预靠点没有足够的转向净空")
        if self.work is not None and not self.near.allows(self.staging, self.work):
            raise ValueError("作业位太靠近亚克力板，或预靠点与作业位之间净空不足")
