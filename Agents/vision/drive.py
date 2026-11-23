"""车体微调闭环入格：臂固定在 (360, 1570)，用底盘前进/后退/左右平移送指尖入格。

单应定棋盘、标签或黑框定指尖沿用 servo 模块。雅可比换成底盘：分别前进和平移
probe_ms，实测指尖在棋盘上的位移（毫米/毫秒）作为两列，再按 J·Δt = 目标 − 指尖
牛顿迭代，把毫秒数发给 move 模块。臂不动、车动，能到臂本身够不到的格。
看不到棋盘角标或末端、或避障把车轮拦下时立即停车，不再盲动。
"""

import argparse
from contextlib import nullcontext
from dataclasses import asdict, dataclass
import json
import math
from time import monotonic

import numpy as np

from .arm import target_xy
from .camera import Mount
from .servo import Servo, ServoConfig, TagNotVisible, clamp_pulses
from .tags import Detector

if "." in __package__:
    from ..move import move as chassis_move
    from ..uno import DEFAULT_PORT, MotionBlocked, open_uno
else:
    from move import move as chassis_move
    from uno import DEFAULT_PORT, MotionBlocked, open_uno

# 11-23 实测：臂停在此脉宽时指尖约在 (100, 167) mm，即 7 格附近。
# 从这一姿态走车，可以把指尖送到 0、1、2、4、5 等臂单独够不到的格。
DRIVE_PULSES = (360, 1570)
# 每个轴的两种动作，索引 0/1 是正/负方向；probe_sign 取 -1 时整对调换。
AXIS_ACTIONS = (("go", "back"), ("left", "right"))


@dataclass(frozen=True)
class DriveConfig:
    probe_ms: int = 300              # 测雅可比的单次底盘动作时长
    probe_sign: tuple = (1, 1)       # 前进、左移为正方向；-1 改为后退/右移
    max_step_ms: int = 700           # 单次走车修正时长上限
    tolerance_mm: float = 5.0
    settle_s: float = 0.4            # 停车后丢弃旧帧的时长
    max_steps: int = 8               # 每格最多迭代步数
    refresh_steps: int = 3           # 每隔几步重测雅可比


@dataclass(frozen=True)
class DriveResult:
    cell: int
    ok: bool
    error_mm: float
    pulses: tuple        # 臂固定的脉宽，走车全程不变
    moves: int           # 下发的底盘动作次数，含雅可比探测
    rms_px: float        # 最后一次角标重投影误差
    note: str = ""       # 未收敛或提前停车的原因


class Driver:
    """复用 Servo 的相机流与 Uno 连接，只把修正量换成底盘毫秒数。"""

    def __init__(self, servo, *, config=DriveConfig(), pulses=DRIVE_PULSES, log=print):
        self.servo = servo
        self.config = config
        self.pulses = clamp_pulses(*pulses, servo.limits)
        self.actions = tuple(pair if sign > 0 else pair[::-1]
                             for pair, sign in zip(AXIS_ACTIONS, config.probe_sign))
        self.log = log

    def _drive(self, axis, duration_ms):
        action = self.actions[axis][0] if duration_ms > 0 else self.actions[axis][1]
        chassis_move(action, abs(duration_ms), link=self.servo.link)
        deadline = monotonic() + self.config.settle_s
        for stamp, _ in self.servo.stream:
            if stamp >= deadline:
                return

    def _probe(self, world):
        """分别前进、左移一次，用指尖位移测出毫米/毫秒两列。"""
        columns, state = [], None
        for axis in (0, 1):
            self._drive(axis, self.config.probe_ms)
            tip, world_now, view = self.servo.observe()
            moved = world_now - world
            if np.linalg.norm(moved) < 1.0:
                raise RuntimeError("底盘动作后末端几乎没动，检查车轮、电池和超声避障")
            columns.append(moved / self.config.probe_ms)
            state = tip, world_now, view
        jacobian = np.column_stack(columns)
        if abs(np.linalg.det(jacobian)) < 1e-3:
            raise RuntimeError("前进与平移在棋盘上几乎同向，雅可比接近奇异，先摆正车头")
        return (jacobian,) + state

    def place(self, cell, target=None):
        """臂固定后用车体闭环把指尖送到该格；看不到标签或避障就停车返回。"""
        if target is None:
            target = self.servo.targets.get(cell)
        if target is None:
            target = target_xy(cell)
        target = np.asarray(target, dtype=float)
        self.servo._move(self.pulses)
        moves = 0
        try:
            tip, world, view = self.servo.observe()
        except TagNotVisible:
            self.log(f"格 {cell}：臂固定位 {self.pulses} 看不到末端标签，未动车")
            return DriveResult(cell, False, float("nan"), self.pulses, moves, float("nan"),
                               "臂固定位看不到末端标签，未动车")
        self.log(f"格 {cell}：目标 ({target[0]:.1f}, {target[1]:.1f}) mm，臂固定 {self.pulses}，"
                 f"指尖 ({world[0]:.1f}, {world[1]:.1f}) mm，角标重投影 {view.rms_px:.2f} px")
        try:
            jacobian = None
            for step in range(self.config.max_steps + 1):
                error = target - world
                distance = float(np.linalg.norm(error))
                if distance <= self.config.tolerance_mm:
                    return DriveResult(cell, True, distance, self.pulses, moves, view.rms_px)
                if step == self.config.max_steps:
                    break
                if jacobian is None or step % self.config.refresh_steps == 0:
                    jacobian, tip, world, view = self._probe(world)
                    moves += 2
                    continue
                step_ms = np.clip(np.linalg.solve(jacobian, error),
                                  -self.config.max_step_ms, self.config.max_step_ms)
                acted = False
                for axis in (0, 1):
                    duration = int(round(step_ms[axis]))
                    if duration == 0:
                        continue
                    self._drive(axis, duration)
                    moves += 1
                    acted = True
                    tip, world, view = self.servo.observe()
                if not acted:
                    return DriveResult(cell, False, float(np.linalg.norm(target - world)),
                                       self.pulses, moves, view.rms_px,
                                       "修正量不足 1 ms，车体分辨率到不了更细")
                self.log(f"格 {cell}：第 {moves} 次动作 → 残余 "
                         f"{np.linalg.norm(target - world):.1f} mm")
        except TagNotVisible:
            self.log(f"格 {cell}：走车后看不到末端标签，已停车")
            return DriveResult(cell, False, float("nan"), self.pulses, moves, view.rms_px,
                               "看不到末端标签，已停车")
        except MotionBlocked:
            self.log(f"格 {cell}：超声避障拦下车体动作，已停车")
            return DriveResult(cell, False, float("nan"), self.pulses, moves, view.rms_px,
                               "超声避障拦下车体动作，已停车")
        return DriveResult(cell, False, float(np.linalg.norm(target - world)),
                           self.pulses, moves, view.rms_px, "迭代次数用完仍未到位")


def drive_cells(cells, *, source, mount=None, port=DEFAULT_PORT, link=None,
                detector=None, servo_config=ServoConfig(), config=DriveConfig(),
                targets=None, pulses=DRIVE_PULSES, release=False, log=print):
    """按顺序走车送完各格。臂全程固定在 pulses，不自动 ARM OFF。"""
    with (nullcontext(link) if link is not None else open_uno(port)) as connection:
        servo = Servo(connection, source, mount, detector=detector, config=servo_config, log=log)
        try:
            if targets:
                servo.targets.update({int(cell): np.asarray(xy, dtype=float)
                                      for cell, xy in targets.items()})
            missing = [cell for cell in cells if cell not in servo.targets]
            if missing:
                servo.measure_targets(range(9))
            driver = Driver(servo, config=config, pulses=pulses, log=log)
            results = []
            for cell in cells:
                result = driver.place(cell)
                results.append(result)
                if not result.ok:
                    residual = "" if math.isnan(result.error_mm) else \
                        f"，残余 {result.error_mm:.1f} mm"
                    log(f"格 {cell} 未完成：{result.note}{residual}")
            if release:
                servo.release()
            return results
        finally:
            servo.close()


def main():
    p = argparse.ArgumentParser(description="臂固定 (360,1570)，用底盘前进/平移把末端送进格心")
    p.add_argument("--cells", type=int, nargs="+", default=[4], choices=range(9))
    p.add_argument("--source", default="0")
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--config", help="安装参数 JSON，缺省按 1280×720 顶摄")
    p.add_argument("--probe", type=int, default=300, help="测雅可比的底盘动作时长 ms")
    p.add_argument("--probe-sign", type=int, nargs=2, choices=(-1, 1), metavar=("前进", "左移"),
                   help="探测方向，-1 改为后退/右移")
    p.add_argument("--max-step", type=int, default=700, help="单次走车修正时长上限 ms")
    p.add_argument("--tolerance", type=float, default=5.0, help="到位误差 mm")
    p.add_argument("--max-steps", type=int, default=8, help="每格迭代上限")
    p.add_argument("--arm", type=int, nargs=2, default=list(DRIVE_PULSES), metavar=("肩µs", "肘µs"),
                   help="固定臂脉宽，默认 360 1570")
    p.add_argument("--targets", help="每格目标 JSON，如 {\"4\": [100.3, 100.0]}")
    p.add_argument("--release", action="store_true", help="结束后 ARM OFF，臂会失去保持力矩")
    p.add_argument("--raw", action="store_true", help="相机用未压缩 YUYV；MJPG 会让小角标读不出")
    args = p.parse_args()

    mount = Mount.load(args.config)
    if args.raw:
        mount.raw = True
    if not mount.image_size:
        mount.image_size = (1280, 720)
    config = DriveConfig(probe_ms=args.probe, probe_sign=tuple(args.probe_sign or (1, 1)),
                         max_step_ms=args.max_step, tolerance_mm=args.tolerance,
                         max_steps=args.max_steps)
    targets = json.load(open(args.targets, encoding="utf-8")) if args.targets else None

    results = drive_cells(args.cells, source=args.source, mount=mount, port=args.port,
                          detector=Detector(("36h11", "36h10")), config=config,
                          targets=targets, pulses=tuple(args.arm), release=args.release)
    for result in results:
        print(json.dumps(asdict(result), ensure_ascii=False))


if __name__ == "__main__":
    main()