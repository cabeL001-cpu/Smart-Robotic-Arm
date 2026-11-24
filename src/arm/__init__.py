"""机械臂接口。规划可独立调用，只有实际发指令时才打开串口。"""

from contextlib import nullcontext
from dataclasses import dataclass, field
import json
from math import ceil, floor, isfinite
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Optional

from .kinematics import Geometry, cell_center, forward, inverse

if "." in __package__:
    from ..uno import DEFAULT_PORT, open_uno, wait_reply
else:
    from uno import DEFAULT_PORT, open_uno, wait_reply


@dataclass(frozen=True)
class ServoCalibration:
    zero_us: float = 1500
    us_per_degree: float = 2000 / 270
    min_us: int = 500
    max_us: int = 2500
    angle_at_max_deg: Optional[float] = None  # 轴在最大位置时，连杆的几何关节角
    retreat_sign: int = -1                    # 从最大位置退回时，几何角变化的符号
    margin_deg: float = 3.0                   # 两端各留的机械余量

    def pulse_limits(self, design_limit=float("inf")):
        """将最后 90°、图纸角限和电气限位取交集，向内取整供 Uno 复用。"""
        if self.angle_at_max_deg is None:
            raise ValueError("缺少 angle_at_max_deg，请先标定该关节的机械最大位置")
        if not (self.retreat_sign in (-1, 1) and 0 <= self.margin_deg < 45
                and isfinite(self.angle_at_max_deg) and isfinite(self.zero_us)
                and isfinite(self.us_per_degree) and self.us_per_degree != 0
                and 500 <= self.min_us <= self.max_us <= 2500):
            raise ValueError("舵机标定的旋向、端点余量、斜率或脉宽范围无效")
        low, high = sorted(self.angle_at_max_deg + self.retreat_sign * distance
                           for distance in (self.margin_deg, 90 - self.margin_deg))
        low, high = max(low, -design_limit), min(high, design_limit)
        if low > high:
            raise ValueError("机械行程与图纸角限没有交集")
        pwm = sorted(self.zero_us + q * self.us_per_degree for q in (low, high))
        bounds = ceil(max(pwm[0], self.min_us)), floor(min(pwm[1], self.max_us))
        if bounds[0] > bounds[1]:
            raise ValueError("机械行程与可用脉宽没有交集")
        return bounds

    def angle_limits(self, design_limit):
        return sorted((us - self.zero_us) / self.us_per_degree
                      for us in self.pulse_limits(design_limit))

    def pulse(self, angle_deg, design_limit=float("inf")):
        low, high = self.pulse_limits(design_limit)
        target = self.zero_us + self.us_per_degree * angle_deg
        if not low - 1e-7 <= target <= high + 1e-7:
            raise ValueError(f"目标 {angle_deg:.2f}° 超出已标定脉宽区间 [{low}, {high}] µs")
        return round(target)


@dataclass(frozen=True)
class ArmConfig:
    geometry: Geometry = field(default_factory=Geometry)
    shoulder: ServoCalibration = field(default_factory=ServoCalibration)
    elbow: ServoCalibration = field(default_factory=ServoCalibration)

    def axes(self):
        return ((self.shoulder, self.geometry.shoulder_limit_deg),
                (self.elbow, self.geometry.elbow_limit_deg))

    def joint_limits(self):
        return [servo.angle_limits(cap) for servo, cap in self.axes()]

    def solve(self, x_mm, y_mm, previous=(0., 0.), geometry=None):
        return inverse(x_mm, y_mm, geometry or self.geometry, previous,
                       joint_limits=self.joint_limits())

    def write_firmware_limits(self, path):
        """同一份实测数据生成固件硬限位，原始 ARM 指令也经过这道检查。"""
        lines = ["#pragma once", "// 由 python -m arm --export-limits 从实车标定生成。"]
        for name, (servo, cap) in zip(("SHOULDER", "ELBOW"), self.axes()):
            low, high = servo.pulse_limits(cap)
            lines.append(f"constexpr int {name}_MIN_US = {low};")
            lines.append(f"constexpr int {name}_MAX_US = {high};")
        target = Path(path)
        output = NamedTemporaryFile(dir=target.parent, delete=False, mode="w", encoding="utf-8")
        temp = Path(output.name)
        try:
            with output:
                output.write("\n".join(lines) + "\n")
            temp.replace(target)
        finally:
            temp.unlink(missing_ok=True)

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(Geometry(**data.get("geometry", {})),
                   ServoCalibration(**data.get("shoulder", {})),
                   ServoCalibration(**data.get("elbow", {})))


def joint_command(angles, config):
    shoulder, elbow = angles
    shoulder_us = config.shoulder.pulse(shoulder, config.geometry.shoulder_limit_deg)
    elbow_us = config.elbow.pulse(elbow, config.geometry.elbow_limit_deg)
    return f"ARM {shoulder_us} {elbow_us}\n".encode("ascii")


def set_joints(angles, *, config=ArmConfig(), port=DEFAULT_PORT, link=None):
    command = joint_command(angles, config)
    with (nullcontext(link) if link is not None else open_uno(port)) as connection:
        connection.write(command)
        wait_reply(connection, b"OK ARM")


def arm(cell, *, config=ArmConfig(), port=DEFAULT_PORT, link=None, previous=(0.0, 0.0)):
    """在图纸的对齐工位瞄准指定格，返回 Uno 已接受的两个关节角。"""
    angles = config.solve(*cell_center(cell), previous=previous)
    set_joints(angles, config=config, port=port, link=link)
    return angles
