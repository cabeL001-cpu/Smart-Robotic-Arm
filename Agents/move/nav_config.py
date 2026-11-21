"""导航场地坐标变换、实测运动参数与闭环阈值。"""

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Optional

from .imu import rotate, wrap


@dataclass(frozen=True)
class Origin:
    """启动系原点在场地中的位姿。两系长度均为米。"""
    x_m: float
    y_m: float
    yaw_rad: float

    def to_field(self, x, y):
        dx, dy = rotate(x, y, self.yaw_rad)
        return self.x_m + dx, self.y_m + dy

    def to_start(self, x, y):
        return rotate(x - self.x_m, y - self.y_m, -self.yaw_rad)

    @classmethod
    def match(cls, x_field, y_field, yaw_field, odom):
        yaw = wrap(yaw_field - odom.yaw_rad)
        dx, dy = rotate(odom.x_m, odom.y_m, yaw)
        return cls(x_field - dx, y_field - dy, yaw)


@dataclass(frozen=True)
class Control:
    coarse_m: float = .18
    tolerance_m: float = .035
    min_pulse_ms: int = 65
    max_pulse_ms: int = 350
    settle_s: float = .25
    scan_step_deg: float = 18.
    min_scan_tags: int = 2
    confirm_frames: int = 3
    agreement_m: float = .12
    agreement_deg: float = 12.
    vision_timeout_s: float = 3.
    timeout_s: float = 150.
    stall_pulses: int = 6
    calibration_s: float = 2.

    def __post_init__(self):
        if not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0
                   for v in vars(self).values()):
            raise ValueError("导航控制参数必须为有限正数")
        counts = (self.min_pulse_ms, self.max_pulse_ms, self.min_scan_tags,
                  self.confirm_frames, self.stall_pulses)
        if (any(type(v) is not int for v in counts) or self.min_pulse_ms > self.max_pulse_ms
                or self.max_pulse_ms > 1000 or self.confirm_frames < 2 or self.scan_step_deg > 45
                or self.agreement_deg >= 90 or self.tolerance_m >= self.coarse_m):
            raise ValueError("请核对导航脉冲、扫描角、确认帧数和粗精定位阈值")


@dataclass(frozen=True)
class NavigationConfig:
    field: Path
    front_mount: Path
    origin: Optional[Origin]
    speeds: dict
    imu_mount_yaw_deg: float
    control: Control

    @classmethod
    def load(cls, path):
        path = Path(path).expanduser()
        data = json.loads(path.read_text(encoding="utf-8"))
        speeds = data["speeds"]
        for action in ("go", "back", "left", "right", "rotate-left", "rotate-right"):
            v = speeds.get(action)
            if not isinstance(v, (float, int)) or isinstance(v, bool) or not math.isfinite(v) or v <= 0:
                raise ValueError(f"请填写实测速率 speeds.{action}（平移 m/s、转动 °/s）")
        start = data.get("start_field_pose")
        origin = None
        if start is not None:
            if len(start) != 3 or not all(map(math.isfinite, start)):
                raise ValueError("start_field_pose 应为 [场地X米,场地Y米,车头角度] 或 null")
            origin = Origin(start[0], start[1], math.radians(start[2]))
        imu_yaw = data.get("imu_mount_yaw_deg", 0.)
        if not math.isfinite(imu_yaw):
            raise ValueError("imu_mount_yaw_deg 需为实测安装角")
        return cls(path.parent / data["field"], path.parent / data["front_mount"], origin,
                   speeds, imu_yaw, Control(**data.get("control", {})))
