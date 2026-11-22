"""末端方块闭环入格：单应定棋盘、Otsu 黑框定指尖、实测雅可比做牛顿修正。

本机 720p 顶摄解不出末端小标签（见 CALIBRATION_PRACTICE 9.3），所以只跟踪
黑白方块的几何中心。每格先用起点脉宽粗定位，再分别给肩、肘加 probe_us 测出
2×2 雅可比（毫米/微秒），按 J·Δp = 目标 − 指尖 迭代；单轴限幅并钳在固件
脉宽内。到位后保持力矩不自动 ARM OFF，竖直臂在悬空位置掉电会砸到棋盘。
"""

import argparse
from contextlib import nullcontext
from dataclasses import asdict, dataclass
import json
import math
from time import monotonic

import cv2
import numpy as np

from .arm import target_xy
from .board import BOARD_LAYOUT, project
from .camera import Mount, frames, live_source
from .tags import Detector

if "." in __package__:
    from ..uno import DEFAULT_PORT, open_uno, wait_reply
else:
    from uno import DEFAULT_PORT, open_uno, wait_reply

# 与 Agents/move/arm_limits.h 保持一致，超出会收到 ERR ARM。
SHOULDER_MIN_US, SHOULDER_MAX_US = 400, 1080
ELBOW_MIN_US, ELBOW_MAX_US = 500, 2500

# 当前视角下末端黑框约 45×41 px，远近和曝光变化留出余量。
TIP_AREA = (700, 2600)
TIP_SIDE = (28, 75)
TIP_RATIO = 1.35
TIP_FILL = 0.6
TIP_CLEAR_PX = 60       # 与印刷角标方块中心的最小距离
MAX_RMS_PX = 8.0        # 角标重投影超过此值视为配错


def clamp_pulses(shoulder_us, elbow_us):
    return (int(round(min(max(shoulder_us, SHOULDER_MIN_US), SHOULDER_MAX_US))),
            int(round(min(max(elbow_us, ELBOW_MIN_US), ELBOW_MAX_US))))


@dataclass(frozen=True)
class BoardView:
    to_world: np.ndarray          # 像素 → 棋盘毫米
    rms_px: float                 # 角标重投影误差
    keys: tuple                   # 参与拟合的标签
    golden_px: tuple              # 印刷角标方块中心，用于排除指尖候选


def board_view(tags):
    """只用角标做最小二乘单应；fit_plane 的 RANSAC 门槛在双码画面会误杀。"""
    keys = [key for key in tags if key in BOARD_LAYOUT]
    if any(key[0] == "printed5" for key in keys):
        keys = [key for key in keys if key[0] == "printed5"]
    if len(keys) < 2 or any(len(tags[key]) != 4 for key in keys):
        return None
    world = np.vstack([BOARD_LAYOUT[key] for key in keys])
    image = np.vstack([tags[key] for key in keys])
    to_pixel, _ = cv2.findHomography(world, image, 0)
    if to_pixel is None:
        return None
    residual = project(to_pixel, world) - image
    rms = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))
    if rms > MAX_RMS_PX:
        return None
    try:
        to_world = np.linalg.inv(to_pixel)
    except np.linalg.LinAlgError:
        return None
    return BoardView(to_world, rms, tuple(keys),
                     tuple(image[i * 4:(i + 1) * 4].mean(axis=0) for i in range(len(keys))))


@dataclass(frozen=True)
class Tip:
    px: tuple            # 黑框方块中心，像素
    side_px: float       # 方块平均边长
    angle_deg: float     # 方块在图像中的转角


def find_tip(frame, golden_px, near=None):
    """Otsu 分割黑框。白芯会让黑框断成两半，先闭运算再接回完整外框。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    _, binary = cv2.threshold(cv2.GaussianBlur(gray, (3, 3), 0), 0, 255,
                              cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if not TIP_AREA[0] < area < TIP_AREA[1]:
            continue
        (x, y), (w, h), angle = cv2.minAreaRect(contour)
        if not (TIP_SIDE[0] < w < TIP_SIDE[1] and TIP_SIDE[0] < h < TIP_SIDE[1]):
            continue
        if max(w, h) / min(w, h) > TIP_RATIO or area / (w * h) < TIP_FILL:
            continue
        if any(math.hypot(x - gx, y - gy) < TIP_CLEAR_PX for gx, gy in golden_px):
            continue
        found.append(Tip((x, y), (w + h) / 2, angle))
    if not found:
        return None
    if near is None:
        return max(found, key=lambda tip: tip.side_px)
    return min(found, key=lambda tip: math.hypot(tip.px[0] - near[0], tip.px[1] - near[1]))


@dataclass(frozen=True)
class ServoConfig:
    probe_us: int = 60                   # 雅可比探测：单轴增加的脉宽
    max_step_us: tuple = (120, 150)      # 单次修正的肩、肘脉宽上限
    tolerance_mm: float = 4.0
    settle_s: float = 0.8                # 发令后丢弃旧帧的时长
    max_steps: int = 6                   # 每格最多迭代步数
    refresh_steps: int = 3               # 每隔几步重测雅可比
    retries: int = 10                    # 缺角标或缺方块时连续丢弃的帧数


@dataclass(frozen=True)
class Placement:
    cell: int
    ok: bool
    error_mm: float
    shoulder_us: int
    elbow_us: int
    moves: int          # 本格下发动作次数，含雅可比探测
    rms_px: float       # 最后一次角标重投影误差


class Servo:
    """复用一次相机流和一个 Uno 连接逐格闭环。link 为 None 时只测量。"""

    def __init__(self, link, source, mount=None, *, detector=None,
                 config=ServoConfig(), log=print):
        if link is not None and not live_source(source):
            raise ValueError("闭环控制需要实时相机，请用相机编号或 /dev/video* 端点")
        self.link = link
        self.mount = mount or Mount(image_size=(1280, 720))
        self.detector = detector or Detector(("36h11", "36h10"))
        self.config = config
        self.log = log
        self.stream = frames(source, self.mount)

    def close(self):
        self.stream.close()

    def _move(self, pulses):
        self.link.write(f"ARM {pulses[0]} {pulses[1]}\n".encode("ascii"))
        wait_reply(self.link, b"OK ARM")
        deadline = monotonic() + self.config.settle_s
        for stamp, _ in self.stream:
            if stamp >= deadline:
                return

    def release(self):
        """解除保持力矩。竖直臂会下垂，确认不会砸到棋盘再调用。"""
        self.link.write(b"ARM OFF\n")
        wait_reply(self.link, b"OK ARM OFF")

    def observe(self, near=None):
        """取当前帧的末端方块及其棋盘毫米坐标。"""
        for _ in range(self.config.retries):
            try:
                _, frame = next(self.stream)
            except StopIteration:
                raise RuntimeError("图像流结束，闭环需要连续帧") from None
            tags = self.detector.detect(frame)
            view = board_view(tags)
            if view is None:
                continue
            tip = find_tip(frame, view.golden_px, near)
            if tip is None:
                continue
            return tip, project(view.to_world, [tip.px])[0], view
        raise RuntimeError("连续多帧未同时看到棋盘角标与末端方块，检查视野与曝光")

    def _probe(self, pulses, tip, world):
        """分别沿单轴加 probe_us，实测指尖位移组成雅可比。"""
        columns, state = [], None
        for axis in (0, 1):
            probe = list(pulses)
            probe[axis] += self.config.probe_us
            probe = clamp_pulses(*probe)
            delta = probe[axis] - pulses[axis]
            if delta == 0:
                raise RuntimeError("关节已顶到固件限位，无法测雅可比，换个起点")
            self._move(probe)
            tip, world_now, view = self.observe(near=tip.px)
            moved = world_now - world
            if np.linalg.norm(moved) < 1.5:
                raise RuntimeError("加脉冲后末端几乎没动，检查舵机供电与接线")
            columns.append(moved / delta)
            state = probe, tip, world_now, view
        jacobian = np.column_stack(columns)
        if abs(np.linalg.det(jacobian)) < 1e-3:
            raise RuntimeError("雅可比接近奇异，末端在可达边界，换个起点")
        return (jacobian,) + state

    def place(self, cell, start):
        """从 start=(肩, 肘) 脉宽出发，把末端中心送进格心。"""
        target = target_xy(cell)
        pulses = clamp_pulses(*start)
        self._move(pulses)
        tip, world, view = self.observe()
        moves = 0
        self.log(f"格 {cell}：起点 {pulses}，末端 ({world[0]:.1f}, {world[1]:.1f}) mm，"
                 f"角标重投影 {view.rms_px:.2f} px")
        jacobian = None
        for step in range(self.config.max_steps + 1):
            error = target - world
            distance = float(np.linalg.norm(error))
            if distance <= self.config.tolerance_mm:
                return Placement(cell, True, distance, pulses[0], pulses[1], moves, view.rms_px)
            if step == self.config.max_steps:
                break
            if jacobian is None or step % self.config.refresh_steps == 0:
                jacobian, pulses, tip, world, view = self._probe(pulses, tip, world)
                moves += 2
                continue
            step_us = np.clip(np.linalg.solve(jacobian, error),
                              -np.asarray(self.config.max_step_us),
                              np.asarray(self.config.max_step_us))
            nxt = clamp_pulses(*(np.asarray(pulses, dtype=float) + step_us))
            if nxt == pulses:
                raise RuntimeError(f"格 {cell}：关节已到固件限位，残余 {distance:.1f} mm")
            self._move(nxt)
            pulses = nxt
            moves += 1
            tip, world, view = self.observe(near=tip.px)
            self.log(f"格 {cell}：第 {moves} 次动作 → {pulses}，"
                     f"残余 {np.linalg.norm(target - world):.1f} mm")
        return Placement(cell, False, float(np.linalg.norm(target - world)),
                         pulses[0], pulses[1], moves, view.rms_px)


def place_cells(cells, *, start, source, mount=None, port=DEFAULT_PORT, link=None,
                detector=None, config=ServoConfig(), release=False, log=print):
    """按顺序送完各格，每格沿用上一格终点作为起点。start 为 (肩, 肘) 脉宽。"""
    with (nullcontext(link) if link is not None else open_uno(port)) as connection:
        servo = Servo(connection, source, mount, detector=detector, config=config, log=log)
        try:
            pulses, placements = clamp_pulses(*start), []
            for cell in cells:
                placement = servo.place(cell, pulses)
                placements.append(placement)
                pulses = placement.shoulder_us, placement.elbow_us
                if not placement.ok:
                    log(f"格 {cell} 未收敛，误差 {placement.error_mm:.1f} mm")
            if release:
                servo.release()
            return placements
        finally:
            servo.close()


def main():
    p = argparse.ArgumentParser(description="顶摄闭环把末端方块送入九宫格格心")
    p.add_argument("--cells", type=int, nargs="+", default=[4], choices=range(9))
    p.add_argument("--start", type=int, nargs=2, metavar=("肩µs", "肘µs"))
    p.add_argument("--source", default="0")
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--config", help="安装参数 JSON，缺省按 1280×720 顶摄")
    p.add_argument("--probe", type=int, default=60, help="雅可比探测脉宽")
    p.add_argument("--tolerance", type=float, default=4.0, help="到位误差 mm")
    p.add_argument("--max-steps", type=int, default=6, help="每格迭代上限")
    p.add_argument("--dry-run", action="store_true", help="只测一次末端位置，不动舵机")
    p.add_argument("--release", action="store_true", help="结束后 ARM OFF，臂会失去保持力矩")
    args = p.parse_args()

    mount = Mount.load(args.config)
    if not mount.image_size:
        mount.image_size = (1280, 720)
    config = ServoConfig(probe_us=args.probe, tolerance_mm=args.tolerance,
                         max_steps=args.max_steps)
    detector = Detector(("36h11", "36h10"))

    if args.dry_run:
        servo = Servo(None, args.source, mount, detector=detector, config=config)
        try:
            tip, world, view = servo.observe()
        finally:
            servo.close()
        print(json.dumps({
            "tip_px": [round(float(v), 1) for v in tip.px],
            "tip_mm": [round(float(v), 2) for v in world],
            "rms_px": round(view.rms_px, 2),
            "cells_mm": {str(cell): round(float(np.linalg.norm(target_xy(cell) - world)), 1)
                         for cell in args.cells},
        }, ensure_ascii=False))
        return

    if not args.start:
        p.error("控制需要 --start 肩肘脉宽")
    placements = place_cells(args.cells, start=tuple(args.start), source=args.source,
                             mount=mount, port=args.port, detector=detector,
                             config=config, release=args.release)
    for placement in placements:
        print(json.dumps(asdict(placement), ensure_ascii=False))


if __name__ == "__main__":
    main()