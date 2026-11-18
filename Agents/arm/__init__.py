"""机械臂接口。规划可独立调用，只有实际发指令时才打开串口。"""

from contextlib import nullcontext
from dataclasses import dataclass, field
import json
from pathlib import Path

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

    def pulse(self, angle_deg):
        pulse = round(self.zero_us + self.us_per_degree * angle_deg)
        if not 500 <= self.min_us <= pulse <= self.max_us <= 2500:
            raise ValueError(f"目标脉宽 {pulse} µs 超出舵机标定范围")
        return pulse


@dataclass(frozen=True)
class ArmConfig:
    geometry: Geometry = field(default_factory=Geometry)
    shoulder: ServoCalibration = field(default_factory=ServoCalibration)
    elbow: ServoCalibration = field(default_factory=ServoCalibration)

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(Geometry(**data.get("geometry", {})),
                   ServoCalibration(**data.get("shoulder", {})),
                   ServoCalibration(**data.get("elbow", {})))


def joint_command(angles, config):
    shoulder, elbow = angles
    if not (abs(shoulder) <= config.geometry.shoulder_limit_deg
            and abs(elbow) <= config.geometry.elbow_limit_deg):
        raise ValueError("目标关节角超限")
    return f"ARM {config.shoulder.pulse(shoulder)} {config.elbow.pulse(elbow)}\n".encode("ascii")


def set_joints(angles, *, config=ArmConfig(), port=DEFAULT_PORT, link=None):
    command = joint_command(angles, config)
    with (nullcontext(link) if link is not None else open_uno(port)) as connection:
        connection.write(command)
        wait_reply(connection, b"OK ARM")


def arm(cell, *, config=ArmConfig(), port=DEFAULT_PORT, link=None, previous=(0.0, 0.0)):
    """在图纸的对齐工位瞄准指定格，返回 Uno 已接受的两个关节角。"""
    angles = inverse(*cell_center(cell), config.geometry, previous)
    set_joints(angles, config=config, port=port, link=link)
    return angles
