"""平面二连杆运动学。位置用毫米，关节角用度，逆时针为正。"""

from dataclasses import dataclass
from math import acos, atan2, cos, degrees, hypot, radians, sin


@dataclass(frozen=True)
class Geometry:
    upper_mm: float = 152.0
    forearm_mm: float = 212.0
    base_x_mm: float = -100.0
    base_y_mm: float = 150.0
    base_yaw_deg: float = 0.0
    shoulder_limit_deg: float = 85.0
    elbow_limit_deg: float = 135.0


def cell_center(cell):
    """格板左下角在框架 (50, 50)。格号从左下到右上按行递增。"""
    if str(cell) not in tuple("012345678"):
        raise ValueError("格号应为 0 至 8")
    row, column = divmod(int(cell), 3)
    pitch = 200.0 / 3
    return 50 + (column + 0.5) * pitch, 50 + (row + 0.5) * pitch


def forward(shoulder_deg, elbow_deg, geometry=Geometry()):
    g = geometry
    first = radians(shoulder_deg + g.base_yaw_deg)
    second = first + radians(elbow_deg)
    return (g.base_x_mm + g.upper_mm * cos(first) + g.forearm_mm * cos(second),
            g.base_y_mm + g.upper_mm * sin(first) + g.forearm_mm * sin(second))


def inverse(x_mm, y_mm, geometry=Geometry(), previous=(0.0, 0.0)):
    """排除超限解，选择距上次关节角最近的肘部构型。"""
    g = geometry
    dx, dy = x_mm - g.base_x_mm, y_mm - g.base_y_mm
    cosine = (dx * dx + dy * dy - g.upper_mm**2 - g.forearm_mm**2) / (2 * g.upper_mm * g.forearm_mm)
    if not -1.0000000001 <= cosine <= 1.0000000001:
        raise ValueError("目标超出机械臂工作半径")
    bend = acos(max(-1.0, min(1.0, cosine)))
    candidates = []
    for elbow in (bend, -bend):
        shoulder = atan2(dy, dx) - atan2(g.forearm_mm * sin(elbow), g.upper_mm + g.forearm_mm * cos(elbow))
        shoulder_deg = (degrees(shoulder) - g.base_yaw_deg + 180) % 360 - 180
        elbow_deg = degrees(elbow)
        if abs(shoulder_deg) <= g.shoulder_limit_deg and abs(elbow_deg) <= g.elbow_limit_deg:
            candidates.append((shoulder_deg, elbow_deg))
    if not candidates:
        raise ValueError("目标超出关节角度范围")
    return min(candidates, key=lambda q: hypot(q[0] - previous[0], q[1] - previous[1]))
