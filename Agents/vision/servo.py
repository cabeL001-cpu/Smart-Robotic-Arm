"""末端闭环入格：单应定棋盘、标签或黑框定指尖、实测雅可比做牛顿修正。

末端标签可解码时直接用四角中心；顶摄读不出小标签时退到黑白方块的几何中心
（见 CALIBRATION_PRACTICE 9.3）。每格目标是棋盘上印刷数字的质心（量出来存成数据，
不用几何公式硬算），量不到时才退回几何格心。

两种算法：

- 带预置位（`place`/`place_cells`）：先摆到实测预置位 CELL_SEEDS 再闭环微调。
  预置位只在车停在 11-23 标定位姿时有效。
- 全闭环（`place_closed`/`place_closed_cells`）：不摆任何预置位，从当前已知脉宽
  出发，靠视觉自适应探测、阻尼最小二乘和末段单向接近收敛；九宫格相对车体位置
  变了也能用。代价是起点必须能看到末端方块，看不到就不动车。

两种都看不到末端标签就停在该处，不再盲动。到位后保持力矩不自动 ARM OFF，
竖直臂悬空掉电会砸到棋盘。
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
from .tags import Detector, describe

if "." in __package__:
    from ..uno import DEFAULT_PORT, open_uno, wait_reply
else:
    from uno import DEFAULT_PORT, open_uno, wait_reply

# 与 Agents/move/arm_limits.h 保持一致，超出会收到 ERR ARM。
# 11-23 实测肩部：≤320 µs 无动作，360–640 流畅，700 起抖动；固件下限放到 50 供扫端点，
# 运行窗口仍取 360–640。肘部固件 50–2700，入格运行窗口保守取 500–2500。
SHOULDER_MIN_US, SHOULDER_MAX_US = 50, 1080
ELBOW_MIN_US, ELBOW_MAX_US = 50, 2700
SHOULDER_RUN_US = (360, 640)
ELBOW_RUN_US = (500, 2500)

# 11-23 实测预置位：先摆到该格脉宽，再由顶摄闭环微调。格 0 的末端码被大臂挡住，
# 顶摄量不到，开环停在预置位即可（用户目视确认居中），闭环一开始就会按“停止”处理。
CELL_SEEDS = {
    0: (500, 640),
    3: (360, 1162),
    6: (463, 1429),
    7: (360, 1570),
    8: (360, 1920),
}


class TagNotVisible(RuntimeError):
    """顶摄连续多帧看不到棋盘角标或末端，闭环不能再盲动。"""

    def __init__(self, message, pulses=None):
        super().__init__(message)
        self.pulses = pulses      # 探测中断时臂实际停住的脉宽

# 角标和末端码的黑边四边形。当前视角下末端码约 55×54 px，强透视时略大。
TIP_AREA = (600, 6000)
TIP_SIDE = (20, 75)
TIP_RATIO = 1.8
# 11-23 实测：末端停在 8 格时，码中心距右下角标中心仅约 49 px，60 px 会误拒；
# 角标自身的黑框中心几乎与角标中心重合，30 px 仍能排除。
TIP_CLEAR_PX = 30       # 与印刷角标方块中心的最小距离
MAX_RMS_PX = 8.0        # 角标重投影超过此值视为配错


def clamp_pulses(shoulder_us, elbow_us,
                 limits=((SHOULDER_MIN_US, SHOULDER_MAX_US), (ELBOW_MIN_US, ELBOW_MAX_US))):
    return (int(round(min(max(shoulder_us, limits[0][0]), limits[0][1]))),
            int(round(min(max(elbow_us, limits[1][0]), limits[1][1]))))


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
    # 排除表按单应投影出全部印刷角标中心：个别角标被末端挡住读不到时仍能排除误锁。
    golden = tuple(project(to_pixel, [BOARD_LAYOUT[key].mean(axis=0)])[0]
                   for key in BOARD_LAYOUT if key[0] == "printed5")
    return BoardView(to_world, rms, tuple(keys), golden)


@dataclass(frozen=True)
class Tip:
    px: tuple            # 黑框方块中心，像素
    side_px: float       # 方块平均边长
    angle_deg: float     # 方块在图像中的转角


def find_tip(frame, golden_px, near=None, view=None):
    """自适应阈值找带黑边的四边形。强透视或反光让标签解不出时，靠外框定位。

    白芯和格线会把黑框拆开，先闭运算再接回；外框补零后的 80×80 图里，
    黑边应占满最外圈，内部则黑白都有——数字、实心黑块和木臂都会落选。
    给了 view 就只收棋盘范围（外扩 20 mm）内的候选，防止机架黑框误锁。
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 51, 8)
    dst = np.float32([[0, 0], [79, 0], [79, 79], [0, 79]])
    found = []
    for width in (3, 5, 7, 9):
        kernel = np.ones((width, width), np.uint8)
        blobs = cv2.morphologyEx(cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel),
                                 cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(blobs, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if not TIP_AREA[0] < cv2.contourArea(contour) < TIP_AREA[1]:
                continue
            hull = cv2.convexHull(contour)
            quad = cv2.approxPolyDP(hull, 0.04 * cv2.arcLength(hull, True), True)
            if len(quad) != 4 or not cv2.isContourConvex(quad):
                continue
            quad = quad.reshape(4, 2).astype(np.float32)
            (x, y), (w, h), angle = cv2.minAreaRect(quad)
            if not (TIP_SIDE[0] < w < TIP_SIDE[1] and TIP_SIDE[0] < h < TIP_SIDE[1]):
                continue
            if max(w, h) / min(w, h) > TIP_RATIO:
                continue
            if any(math.hypot(x - gx, y - gy) < TIP_CLEAR_PX for gx, gy in golden_px):
                continue
            if view is not None:
                world = project(view.to_world, [quad.mean(axis=0)])[0]
                if not (-20 <= world[0] <= 220 and -20 <= world[1] <= 220):
                    continue
            patch = cv2.warpPerspective(gray, cv2.getPerspectiveTransform(quad, dst), (80, 80))
            _, bits = cv2.threshold(patch, 0, 1, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
            border = np.r_[bits[:10].ravel(), bits[-10:].ravel(), bits[:, :10].ravel(), bits[:, -10:].ravel()]
            # 11-23 实测：末端码保护膜有皱褶/反光，外圈边框均值只有 0.72，门槛放到 0.65。
            if border.mean() < 0.65 or not 0.15 < bits[10:-10, 10:-10].mean() < 0.85:
                continue
            found.append(Tip(tuple(quad.mean(axis=0)), (w + h) / 2, angle))
    if not found:
        return None
    if near is None:
        return max(found, key=lambda tip: tip.side_px)
    return min(found, key=lambda tip: math.hypot(tip.px[0] - near[0], tip.px[1] - near[1]))


# 印刷数字：本板数字约 23×16 px，面积 120–260 px；旋转后长宽比仍小于 2.5。
DIGIT_AREA = (120, 1800)
DIGIT_SIDE = (8, 55)
DIGIT_RATIO = 2.5
DIGIT_MATCH_MM = 8.0    # 数字质心与几何格心的最大偏差，超过视为误检


def digit_targets(frame, view, cells=range(9)):
    """用印刷数字的质心当格心。只做最近邻配对，不用几何公式定目标。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    count, _, stats, cents = cv2.connectedComponentsWithStats(binary, 8)
    candidates = []
    for index in range(1, count):
        width, height = stats[index, cv2.CC_STAT_WIDTH], stats[index, cv2.CC_STAT_HEIGHT]
        area = stats[index, cv2.CC_STAT_AREA]
        if not DIGIT_AREA[0] < area < DIGIT_AREA[1]:
            continue
        if not (DIGIT_SIDE[0] < width < DIGIT_SIDE[1] and DIGIT_SIDE[0] < height < DIGIT_SIDE[1]):
            continue
        if max(width, height) / min(width, height) > DIGIT_RATIO:
            continue
        world = project(view.to_world, [cents[index]])[0]
        if not (5 < world[0] < 195 and 5 < world[1] < 195):
            continue
        candidates.append(np.asarray(world, dtype=float))
    targets = {}
    for cell in cells:
        ideal = np.asarray(target_xy(cell), dtype=float)
        near = min(candidates, key=lambda world: np.linalg.norm(world - ideal), default=None)
        if near is not None and np.linalg.norm(near - ideal) <= DIGIT_MATCH_MM:
            targets[cell] = near
    return targets


@dataclass(frozen=True)
class ServoConfig:
    probe_us: int = 60                   # 雅可比探测：单轴增加的脉宽
    probe_sign: tuple = (1, 1)           # 肩、肘探测方向；-1 表示往反方向探，避免出棋盘
    max_step_us: tuple = (120, 150)      # 单次修正的肩、肘脉宽上限
    tolerance_mm: float = 4.0
    settle_s: float = 0.8                # 发令后丢弃旧帧的时长
    max_steps: int = 6                   # 每格最多迭代步数
    refresh_steps: int = 3               # 每隔几步重测雅可比
    retries: int = 10                    # 缺角标或缺方块时连续丢弃的帧数
    # 本轮允许的脉宽窗口。机械上限可能低于固件限位，可按实测收窄。
    shoulder_us: tuple = SHOULDER_RUN_US
    elbow_us: tuple = ELBOW_RUN_US


@dataclass(frozen=True)
class ClosedConfig:
    """全闭环入格参数：不依赖预置位，所有动作都由视觉反馈决定。"""
    probe_us: int = 60                   # 起始探测脉宽，位移太小会自动加大
    probe_max_us: int = 240              # 自适应探测的脉宽上限
    probe_min_mm: float = 1.5            # 探测位移下限，低于则加大探测
    probe_sign: tuple = (1, 1)           # 首选的探测方向；撞窗口自动反向
    samples: int = 3                     # 每次观测取几帧，用中位数抗噪声
    max_spread_mm: float = 1.5           # 多帧离散上限，超过视为还在动或误检
    gain: float = 0.9                    # 世界坐标步长的阻尼，小于 1 防越过
    max_move_mm: float = 6.0             # 单步世界位移上限，保持线性化有效
    max_step_us: tuple = (120, 150)      # 修正阶段单轴脉宽步长上限
    final_step_us: tuple = (40, 50)      # 末段接近的单轴脉宽步长上限
    tolerance_mm: float = 4.0            # 实测有效重复精度 3–4 mm；2 mm 会反复探测直至丢标签
    approach_mm: float = 0.0             # 默认关闭；目标靠近可达边缘时退让反而恶化杠杆
    approach_dir: tuple = (1.0, 1.0)     # 末段退让方向，棋盘坐标
    approach_tolerance_mm: float = 1.5
    approach_steps: int = 4              # 末段退让阶段的迭代上限
    hold_frames: int = 3                 # 到位后连续复核的观测次数
    max_steps: int = 18
    refresh_steps: int = 10              # 每隔几步重测雅可比；探测会扰动工作点，不宜过密
    shoulder_us: tuple = SHOULDER_RUN_US
    elbow_us: tuple = ELBOW_RUN_US


@dataclass(frozen=True)
class Placement:
    cell: int
    ok: bool
    error_mm: float
    shoulder_us: int
    elbow_us: int
    moves: int          # 本格下发动作次数，含雅可比探测
    rms_px: float       # 最后一次角标重投影误差
    note: str = ""      # 未收敛或提前停止的原因


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
        self.targets = {}           # 格号 → 棋盘毫米目标，缺省用印刷数字质心
        self.pulses = None          # 最近一次下发的脉宽，用于失败时报告停车位置

    @property
    def limits(self):
        return self.config.shoulder_us, self.config.elbow_us

    def close(self):
        self.stream.close()

    def measure_targets(self, cells=range(9)):
        """在当前位置量印刷数字的质心，作为缺省格心；看不到的格保留几何格心。"""
        for _ in range(self.config.retries):
            try:
                _, frame = next(self.stream)
            except StopIteration:
                raise RuntimeError("图像流结束，闭环需要连续帧") from None
            tags = self.detector.detect(frame)
            view = board_view(tags)
            if view is None:
                continue
            found = digit_targets(frame, view, cells)
            if not found:
                continue
            self.targets.update(found)
            self.log("数字目标：" + "，".join(
                f"{cell} ({x:.1f}, {y:.1f}) mm" for cell, (x, y) in sorted(found.items())))
            missing = [cell for cell in cells if cell not in found]
            if missing:
                self.log(f"格 {missing} 没量到印刷数字，退用几何格心")
            return found
        raise RuntimeError("连续多帧没看到棋盘角标，无法测量数字位置")

    def _move(self, pulses):
        self.link.write(f"ARM {pulses[0]} {pulses[1]}\n".encode("ascii"))
        wait_reply(self.link, b"OK ARM")
        self.pulses = (int(pulses[0]), int(pulses[1]))
        deadline = monotonic() + self.config.settle_s
        for stamp, _ in self.stream:
            if stamp >= deadline:
                return

    def release(self):
        """解除保持力矩。竖直臂会下垂，确认不会砸到棋盘再调用。"""
        self.link.write(b"ARM OFF\n")
        wait_reply(self.link, b"OK ARM OFF")

    def observe(self, near=None):
        """取当前帧的末端位置及其棋盘毫米坐标。"""
        last_tags, last_view, last_tip = {}, False, False
        for _ in range(self.config.retries):
            try:
                _, frame = next(self.stream)
            except StopIteration:
                raise RuntimeError("图像流结束，闭环需要连续帧") from None
            tags = self.detector.detect(frame)
            last_tags = tags
            view = board_view(tags)
            if view is None:
                last_view, last_tip = False, False
                continue
            last_view = True
            tip = self._tag_tip(tags) or find_tip(frame, view.golden_px, near, view)
            last_tip = tip is not None
            if tip is None:
                continue
            return tip, project(view.to_world, [tip.px])[0], view
        missing = [text for ok, text in ((last_view, "棋盘角标不足"),
                                         (last_tip, "末端不可见")) if not ok]
        raise TagNotVisible(
            f"连续 {self.config.retries} 帧未同时看到棋盘角标与末端方块"
            f"（{'；'.join(missing) or '同帧未同时满足'}；最近一帧检测到 "
            f"{describe(last_tags)}），检查视野与曝光")

    def _tag_tip(self, tags):
        """末端标签可解码时直接取四角中心，比黑框方块稳定。"""
        quad = tags.get((self.mount.tip_family, self.mount.tip_id))
        if quad is None:
            return None
        sides = np.linalg.norm(np.roll(quad, -1, axis=0) - quad, axis=1)
        edge = quad[1] - quad[0]
        return Tip(tuple(quad.mean(axis=0)), float(sides.mean()),
                   math.degrees(math.atan2(edge[1], edge[0])))

    def _probe(self, pulses, tip, world):
        """分别沿单轴按 probe_sign 加 probe_us，实测指尖位移组成雅可比。

        探测方向撞到脉宽窗口时自动反向，避免截断后的步长把雅可比带偏。
        """
        columns, state = [], None
        for axis in (0, 1):
            probe = list(pulses)
            sign = self.config.probe_sign[axis]
            probe[axis] += sign * self.config.probe_us
            if not self.limits[axis][0] <= probe[axis] <= self.limits[axis][1]:
                sign = -sign
                probe[axis] = pulses[axis] + sign * self.config.probe_us
            probe = clamp_pulses(*probe, self.limits)
            delta = probe[axis] - pulses[axis]
            if delta == 0:
                raise RuntimeError("关节已顶到脉宽限位，无法测雅可比，换个起点")
            self._move(probe)
            try:
                tip, world_now, view = self.observe(near=tip.px)
            except TagNotVisible as exc:
                raise TagNotVisible(str(exc), tuple(probe)) from None
            moved = world_now - world
            if np.linalg.norm(moved) < 1.5:
                raise RuntimeError("加脉冲后末端几乎没动，检查舵机供电与接线")
            columns.append(moved / delta)
            state = probe, tip, world_now, view
        jacobian = np.column_stack(columns)
        if abs(np.linalg.det(jacobian)) < 1e-3:
            raise RuntimeError("雅可比接近奇异，末端在可达边界，换个起点")
        return (jacobian,) + state

    def place(self, cell, start=None, target=None):
        """先摆到该格预置位（缺省 CELL_SEEDS），再闭环微调；看不到标签就停。"""
        if target is None:
            target = self.targets.get(cell)
        if target is None:
            target = target_xy(cell)
        target = np.asarray(target, dtype=float)
        if start is None:
            start = CELL_SEEDS.get(cell)
        if start is None:
            raise ValueError(f"格 {cell} 没有预置脉宽，请给 start")
        pulses = clamp_pulses(*start, self.limits)
        self._move(pulses)
        moves = 0
        try:
            tip, world, view = self.observe()
        except TagNotVisible:
            self.log(f"格 {cell}：预置位 {pulses} 看不到末端标签，停在预置位")
            return Placement(cell, False, float("nan"), pulses[0], pulses[1], moves,
                             float("nan"), "预置位看不到末端标签，停在预置位")
        self.log(f"格 {cell}：目标 ({target[0]:.1f}, {target[1]:.1f}) mm，起点 {pulses}，"
                 f"末端 ({world[0]:.1f}, {world[1]:.1f}) mm，角标重投影 {view.rms_px:.2f} px")
        jacobian = None
        for step in range(self.config.max_steps + 1):
            error = target - world
            distance = float(np.linalg.norm(error))
            if distance <= self.config.tolerance_mm:
                return Placement(cell, True, distance, pulses[0], pulses[1], moves, view.rms_px)
            if step == self.config.max_steps:
                break
            try:
                if jacobian is None or step % self.config.refresh_steps == 0:
                    jacobian, pulses, tip, world, view = self._probe(pulses, tip, world)
                    moves += 2
                    continue
                step_us = np.clip(np.linalg.solve(jacobian, error),
                                  -np.asarray(self.config.max_step_us),
                                  np.asarray(self.config.max_step_us))
                nxt = clamp_pulses(*(np.asarray(pulses, dtype=float) + step_us), self.limits)
                if nxt == pulses:
                    raise RuntimeError(f"格 {cell}：关节已到脉宽限位，残余 {distance:.1f} mm")
                self._move(nxt)
                pulses = nxt
                moves += 1
                tip, world, view = self.observe(near=tip.px)
            except TagNotVisible as exc:
                stopped = exc.pulses or pulses
                self.log(f"格 {cell}：动作 {moves} 次后看不到末端标签，停在 {stopped}")
                return Placement(cell, False, float("nan"), stopped[0], stopped[1], moves,
                                 view.rms_px, "看不到末端标签，停在当前位")
            self.log(f"格 {cell}：第 {moves} 次动作 → {pulses}，"
                     f"残余 {np.linalg.norm(target - world):.1f} mm")
        return Placement(cell, False, float(np.linalg.norm(target - world)),
                         pulses[0], pulses[1], moves, view.rms_px, "迭代次数用完仍未到位")

    # ---- 全闭环入格：不摆预置位，所有动作都由视觉反馈决定 ----

    def _clamp_to_window(self, pulses, config):
        limits = ((max(config.shoulder_us[0], SHOULDER_MIN_US),
                   min(config.shoulder_us[1], SHOULDER_MAX_US)),
                  (max(config.elbow_us[0], ELBOW_MIN_US),
                   min(config.elbow_us[1], ELBOW_MAX_US)))
        return clamp_pulses(float(pulses[0]), float(pulses[1]), limits)

    def _observe_median(self, config, near=None):
        """连取多帧取中位数，抑制标签抖动；离散太大说明臂还在动或误检。

        36h10 解不出时黑框回退会在同一位置给出两个相差约 10 px 的候选，
        逐帧独立选“最大候选”会在两者之间跳。第一帧选定后让后续帧按 near
        贴住它，保证同组样本来自同一个候选。
        """
        last = None
        for _ in range(3):
            samples = []
            anchor = near
            try:
                for _ in range(config.samples):
                    sample = self.observe(anchor)
                    samples.append(sample)
                    anchor = sample[0].px
            except TagNotVisible as exc:
                last = exc
                continue
            worlds = np.array([world for _, world, _ in samples])
            center = np.median(worlds, axis=0)
            spread = float(np.max(np.linalg.norm(worlds - center, axis=1)))
            if spread <= config.max_spread_mm:
                index = int(np.argmin(np.linalg.norm(worlds - center, axis=1)))
                tip, _, view = samples[index]
                return tip, center, view
        if last is not None:
            raise last
        raise RuntimeError("末端读数跳动过大，疑似臂还在动或误检，已停")

    def _probe_axis(self, pulses, axis, tip, world, view, config):
        """沿单轴自适应探测：位移低于阈值就加大脉宽重试，偏出窗口自动换向。"""
        limits = ((max(config.shoulder_us[0], SHOULDER_MIN_US),
                   min(config.shoulder_us[1], SHOULDER_MAX_US)),
                  (max(config.elbow_us[0], ELBOW_MIN_US),
                   min(config.elbow_us[1], ELBOW_MAX_US)))
        low, high = limits[axis]
        base_world, base_pulse = world, pulses[axis]
        sign, amplitude, used = config.probe_sign[axis], config.probe_us, 0
        for _ in range(4):
            guess = base_pulse + sign * amplitude
            if not low <= guess <= high:
                sign = -sign
                guess = base_pulse + sign * amplitude
            guess = int(round(min(max(guess, low), high)))
            delta = guess - base_pulse
            if abs(delta) < 15:
                raise RuntimeError("可用脉宽窗口太窄，无法测雅可比，先松开行程")
            nxt = list(pulses)
            nxt[axis] = guess
            nxt = tuple(nxt)
            self._move(nxt)
            used += 1
            try:
                tip, world, view = self._observe_median(config, near=tip.px)
            except TagNotVisible:
                raise TagNotVisible("探测时看不到末端方块", nxt) from None
            moved = world - base_world
            if float(np.linalg.norm(moved)) >= config.probe_min_mm:
                return moved / delta, nxt, tip, world, view, used
            self.log(f"关节 {axis} 加 {abs(delta)} µs 只动了 "
                     f"{np.linalg.norm(moved):.1f} mm，加大探测")
            amplitude = min(config.probe_max_us, int(round(amplitude * 1.6)))
        raise RuntimeError("加大探测脉宽后末端仍几乎不动，检查舵机供电、接线或卡滞")

    def _probe_closed(self, pulses, tip, world, view, config):
        """顺序测两轴雅可比，测完回到基准脉宽，探测不改动工作点。

        每列都相对该次动作前的实测位置计算，不受另一轴回程污染；单轴测完先回
        基准脉宽再测下一轴，否则探测的 +probe_us 会让臂每轮向前爬一个探测步。
        """
        base = tuple(int(v) for v in pulses)
        columns, used = [], 0
        for axis in (0, 1):
            column, probed, tip, world, view, count = self._probe_axis(
                base, axis, tip, world, view, config)
            columns.append(column)
            used += count
            if probed != base:
                self._move(base)
                used += 1
                try:
                    tip, world, view = self._observe_median(config, near=tip.px)
                except TagNotVisible:
                    raise TagNotVisible("探测回位时看不到末端方块", base) from None
        jacobian = np.column_stack(columns)
        singular = np.linalg.svd(jacobian, compute_uv=False)
        if singular[-1] < 1e-4 or singular[-1] < 0.01 * singular[0]:
            raise RuntimeError("雅可比接近奇异，末端在可达边界，先把臂移到棋盘上方再试")
        return jacobian, pulses, tip, world, view, used

    def _servo_phase(self, aim, *, tolerance_mm, budget, final, pulses, tip, world, view,
                     jacobian, moves, config):
        """阻尼最小二乘把末端迭代到 aim 的 tolerance 内；返回更新后的状态。"""
        best, stale = float("inf"), 0
        for step in range(budget):
            error = aim - world
            distance = float(np.linalg.norm(error))
            if distance <= tolerance_mm:
                return True, "", pulses, tip, world, view, jacobian, moves
            try:
                if jacobian is None or (step > 0 and step % config.refresh_steps == 0):
                    jacobian, pulses, tip, world, view, count = self._probe_closed(
                        pulses, tip, world, view, config)
                    moves += count
                    self.log(f"雅可比探测完成，当前 ({world[0]:.1f}, {world[1]:.1f}) mm，"
                             f"残余 {np.linalg.norm(aim - world):.1f} mm")
                    # 探测本身会扰动工作点，不算“没有进步”，否则末段容易被误判停滞
                    continue
                if distance + 0.2 < best:
                    best, stale = distance, 0
                else:
                    stale += 1
                    if stale >= 3:
                        return (False, f"残余不再下降（{distance:.1f} mm），"
                                "目标可能不可达或卡在脉宽限位",
                                pulses, tip, world, view, jacobian, moves)
                step_world = error * config.gain
                length = float(np.linalg.norm(step_world))
                if length > config.max_move_mm:
                    step_world *= config.max_move_mm / length
                dpulses = np.linalg.lstsq(jacobian, step_world, rcond=None)[0]
                cap = np.asarray(config.final_step_us if final else config.max_step_us,
                                 dtype=float)
                dpulses = np.clip(dpulses, -cap, cap)
                nxt = self._clamp_to_window(
                    np.asarray(pulses, dtype=float) + dpulses, config)
                if nxt == pulses:
                    return (False, f"关节已到脉宽限位，残余 {distance:.1f} mm",
                            pulses, tip, world, view, jacobian, moves)
                predicted = jacobian @ (np.asarray(nxt, dtype=float)
                                        - np.asarray(pulses, dtype=float))
                before = world
                self._move(nxt)
                pulses = nxt
                moves += 1
                tip, world, view = self._observe_median(config, near=tip.px)
                if (np.linalg.norm((world - before) - predicted)
                        > max(3.0, 0.6 * float(np.linalg.norm(predicted)))):
                    jacobian = None          # 回差或误检，下次先重测
            except TagNotVisible as exc:
                raise TagNotVisible(str(exc), exc.pulses or pulses) from None
        distance = float(np.linalg.norm(aim - world))
        return (False, f"{budget} 步内未进入 {tolerance_mm:.1f} mm（残余 {distance:.1f} mm）",
                pulses, tip, world, view, jacobian, moves)

    def _verify(self, target, pulses, tip, world, view, jacobian, moves, config):
        """到位后连续多帧复核，防止回差或误检造成假到位。"""
        for _ in range(2):
            good, error = 0, float(np.linalg.norm(target - world))
            for _ in range(config.hold_frames + 2):
                tip, world, view = self._observe_median(config, near=tip.px)
                error = float(np.linalg.norm(target - world))
                good = good + 1 if error <= config.tolerance_mm else 0
                if good >= config.hold_frames:
                    return True, error, pulses, tip, world, view, jacobian, moves
            ok, reason, pulses, tip, world, view, jacobian, moves = self._servo_phase(
                target, tolerance_mm=config.tolerance_mm, budget=4, final=True,
                pulses=pulses, tip=tip, world=world, view=view, jacobian=jacobian,
                moves=moves, config=config)
            if not ok:
                return False, error, pulses, tip, world, view, jacobian, moves
        error = float(np.linalg.norm(target - world))
        return error <= config.tolerance_mm, error, pulses, tip, world, view, jacobian, moves

    def place_closed(self, cell, *, start, target=None, config=ClosedConfig()):
        """全闭环入格：从当前已知脉宽 start 直接视觉伺服到目标，不摆预置位。

        起点看不到末端方块就一次也不动；九宫格相对车体位置变了（车没回到标定
        位姿）也能用。approach_mm 大于 0 时末段先从目标外侧该距离处接近，让
        最后一段的回差方向一致；默认 0（关闭，近可达边缘时退让会恶化杠杆）。
        最后用连续多帧复核到位。
        """
        if target is None:
            target = self.targets.get(cell)
        if target is None:
            target = target_xy(cell)
        target = np.asarray(target, dtype=float)
        pulses = self._clamp_to_window(np.asarray(start, dtype=float), config)
        self.pulses = pulses
        moves, jacobian = 0, None
        try:
            tip, world, view = self._observe_median(config)
        except TagNotVisible:
            self.log(f"格 {cell}：起点看不到末端方块，未动车")
            return Placement(cell, False, float("nan"), pulses[0], pulses[1], 0,
                             float("nan"), "起点看不到末端方块，未动车")
        except RuntimeError as exc:
            return Placement(cell, False, float("nan"), pulses[0], pulses[1], 0,
                             float("nan"), str(exc))
        self.log(f"格 {cell}：目标 ({target[0]:.1f}, {target[1]:.1f}) mm，起点 {pulses}，"
                 f"末端 ({world[0]:.1f}, {world[1]:.1f}) mm，角标重投影 {view.rms_px:.2f} px")
        try:
            if config.approach_mm > 0:
                direction = np.asarray(config.approach_dir, dtype=float)
                direction = direction / np.linalg.norm(direction)
                back = target + direction * config.approach_mm
                ok, reason, pulses, tip, world, view, jacobian, moves = self._servo_phase(
                    back, tolerance_mm=config.approach_tolerance_mm,
                    budget=config.approach_steps, final=False,
                    pulses=pulses, tip=tip, world=world, view=view, jacobian=jacobian,
                    moves=moves, config=config)
                if not ok:
                    self.log(f"格 {cell}：外侧接近不完整（{reason}），直接从当前位修正")
            ok, reason, pulses, tip, world, view, jacobian, moves = self._servo_phase(
                target, tolerance_mm=config.tolerance_mm, budget=config.max_steps,
                final=True, pulses=pulses, tip=tip, world=world, view=view,
                jacobian=jacobian, moves=moves, config=config)
            if not ok:
                return Placement(cell, False, float(np.linalg.norm(target - world)),
                                 pulses[0], pulses[1], moves, view.rms_px, reason)
            ok, error, pulses, tip, world, view, jacobian, moves = self._verify(
                target, pulses, tip, world, view, jacobian, moves, config)
            if not ok:
                return Placement(cell, False, error, pulses[0], pulses[1], moves,
                                 view.rms_px, "到位后复核仍超出容差")
            self.log(f"格 {cell}：全闭环到位，残余 {error:.2f} mm，共 {moves} 次动作")
            return Placement(cell, True, error, pulses[0], pulses[1], moves, view.rms_px)
        except TagNotVisible as exc:
            stopped = exc.pulses or self.pulses or pulses
            self.log(f"格 {cell}：动作 {moves} 次后看不到末端方块，停在 {stopped}")
            return Placement(cell, False, float("nan"), stopped[0], stopped[1], moves,
                             float("nan"), "看不到末端方块，停在当前脉宽")
        except TimeoutError:
            stopped = self.pulses or pulses
            self.log(f"格 {cell}：Uno 回执超时，停在最后一次指令 {stopped}")
            return Placement(cell, False, float("nan"), stopped[0], stopped[1], moves,
                             float("nan"), "Uno 回执超时，臂停在最后一次指令位")
        except OSError as exc:
            # 蓝牙 SPP 掉线时 pyserial 抛 SerialException（OSError 子类）。
            # 只有回执确认过的脉宽才可信，失败那一条指令可能没到 Uno。
            stopped = self.pulses or pulses
            self.log(f"格 {cell}：Uno 通信异常（{exc}），停在最后一次确认位 {stopped}")
            return Placement(cell, False, float("nan"), stopped[0], stopped[1], moves,
                             float("nan"), "Uno 通信异常，臂停在最后一次确认位")
        except RuntimeError as exc:
            self.log(f"格 {cell}：全闭环中止：{exc}")
            return Placement(cell, False, float("nan"), pulses[0], pulses[1], moves,
                             view.rms_px, str(exc))


def place_cells(cells, *, start=None, source, mount=None, port=DEFAULT_PORT, link=None,
                detector=None, config=ServoConfig(), targets=None, release=False, log=print):
    """按顺序送完各格：每格先摆到 CELL_SEEDS 预置位，再闭环微调。

    没有预置位的格沿用上一格终点；start 仅作为第一格的兜底起点。
    看不到末端标签时停在该格当前位，不再盲动，再接着送下一格。
    """
    with (nullcontext(link) if link is not None else open_uno(port)) as connection:
        servo = Servo(connection, source, mount, detector=detector, config=config, log=log)
        try:
            pulses, placements = (tuple(start) if start else None), []
            if targets:
                servo.targets.update({int(cell): np.asarray(xy, dtype=float)
                                      for cell, xy in targets.items()})
            missing = [cell for cell in cells if cell not in servo.targets]
            if missing:
                servo.measure_targets(range(9))
            for cell in cells:
                placement = servo.place(cell, CELL_SEEDS.get(cell) or pulses)
                placements.append(placement)
                pulses = placement.shoulder_us, placement.elbow_us
                if not placement.ok:
                    residual = "" if math.isnan(placement.error_mm) else \
                        f"，残余 {placement.error_mm:.1f} mm"
                    log(f"格 {cell} 未完成：{placement.note}{residual}")
            if release:
                servo.release()
            return placements
        finally:
            servo.close()


def place_closed_cells(cells, *, start=None, source, mount=None, port=DEFAULT_PORT, link=None,
                       detector=None, config=ClosedConfig(), targets=None, release=False,
                       log=print):
    """全闭环逐格：不摆 CELL_SEEDS，第一格须给当前臂脉宽 start，之后沿用上一格终点。

    每一格都从上一格停下的脉宽直接视觉伺服；目标仍按当前画面里的印刷数字质心。
    起点看不到末端方块时不动车，返回未完成，再接着送下一格。
    """
    if start is None:
        raise ValueError("全闭环入格需要当前臂脉宽作为起点，请给 start（CLI: --start 肩µs 肘µs）")
    with (nullcontext(link) if link is not None else open_uno(port)) as connection:
        servo = Servo(connection, source, mount, detector=detector, log=log)
        try:
            if targets:
                servo.targets.update({int(cell): np.asarray(xy, dtype=float)
                                      for cell, xy in targets.items()})
            missing = [cell for cell in cells if cell not in servo.targets]
            if missing:
                servo.measure_targets(range(9))
            pulses, placements = tuple(start), []
            for cell in cells:
                placement = servo.place_closed(cell, start=pulses, config=config)
                placements.append(placement)
                pulses = placement.shoulder_us, placement.elbow_us
                if not placement.ok:
                    residual = "" if math.isnan(placement.error_mm) else \
                        f"，残余 {placement.error_mm:.1f} mm"
                    log(f"格 {cell} 未完成：{placement.note}{residual}")
                    if "超时" in placement.note or "通信" in placement.note:
                        log("Uno 通信中断后臂实际位置不确定，停止继续送格")
                        break
            if release:
                servo.release()
            return placements
        finally:
            servo.close()


def main():
    p = argparse.ArgumentParser(description="顶摄闭环把末端方块送入九宫格格心")
    p.add_argument("--cells", type=int, nargs="+", default=[4], choices=range(9))
    p.add_argument("--start", type=int, nargs=2, metavar=("肩µs", "肘µs"),
                   help="起点；缺省每格先用 CELL_SEEDS 预置位。--closed 时是当前臂脉宽，必须给出")
    p.add_argument("--closed", action="store_true",
                   help="全闭环：不摆预置位，从 --start 当前脉宽直接视觉伺服（九宫格位移时用）")
    p.add_argument("--source", default="0")
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--config", help="安装参数 JSON，缺省按 1280×720 顶摄")
    p.add_argument("--probe", type=int, default=60, help="雅可比探测起始脉宽")
    p.add_argument("--probe-max", type=int, default=240, help="全闭环自适应探测脉宽上限")
    p.add_argument("--probe-sign", type=int, nargs=2, choices=(-1, 1), metavar=("肩", "肘"),
                   help="探测方向，取 -1 往脉宽减小方向探，避开棋盘角标时用")
    p.add_argument("--tolerance", type=float, default=None,
                   help="到位误差 mm，缺省 4.0（两种闭环一致，实测有效重复精度）")
    p.add_argument("--max-steps", type=int, default=None,
                   help="每格迭代上限，缺省预置位闭环 6、全闭环 18（跨格长距离需 30 以上）")
    p.add_argument("--approach-mm", type=float, default=0.0,
                   help="全闭环末段退让距离，默认 0（关闭）；贴近可达边缘时不要开")
    p.add_argument("--hold", type=int, default=3, help="全闭环到位后连续复核次数")
    p.add_argument("--shoulder", type=int, nargs=2, metavar=("最小µs", "最大µs"),
                   help="肩部脉宽窗口，默认 360–640（实测机械可用窗口）")
    p.add_argument("--elbow", type=int, nargs=2, metavar=("最小µs", "最大µs"),
                   help="肘部脉宽窗口，默认 500–2500")
    p.add_argument("--dry-run", action="store_true", help="只测一次末端位置，不动舵机")
    p.add_argument("--measure", action="store_true", help="只量印刷数字位置并输出 JSON，不动舵机")
    p.add_argument("--targets", help="每格目标 JSON，如 {\"6\": [33.9, 165.7]}")
    p.add_argument("--release", action="store_true", help="结束后 ARM OFF，臂会失去保持力矩")
    p.add_argument("--raw", action="store_true", help="相机用未压缩 YUYV；MJPG 会让小角标读不出")
    args = p.parse_args()

    mount = Mount.load(args.config)
    if args.raw:
        mount.raw = True
    if not mount.image_size:
        mount.image_size = (1280, 720)
    windows = {}
    if args.shoulder:
        windows["shoulder_us"] = tuple(args.shoulder)
    if args.elbow:
        windows["elbow_us"] = tuple(args.elbow)
    config = ServoConfig(probe_us=args.probe,
                         tolerance_mm=args.tolerance if args.tolerance is not None else 4.0,
                         max_steps=args.max_steps if args.max_steps is not None else 6,
                         probe_sign=tuple(args.probe_sign or (1, 1)), **windows)
    closed_config = ClosedConfig(probe_us=args.probe, probe_max_us=args.probe_max,
                                 probe_sign=tuple(args.probe_sign or (1, 1)),
                                 tolerance_mm=(args.tolerance if args.tolerance is not None
                                               else 4.0),
                                 max_steps=args.max_steps if args.max_steps is not None else 18,
                                 approach_mm=args.approach_mm, hold_frames=args.hold,
                                 **windows)
    detector = Detector(("36h11", "36h10"))
    targets = json.load(open(args.targets, encoding="utf-8")) if args.targets else None

    if args.closed and not args.start and not (args.dry_run or args.measure):
        p.error("--closed 需要 --start 当前肩肘脉宽：固件只能下发绝对脉宽，读不回当前位置")
    if args.closed and args.dry_run:
        p.error("--closed 是动车模式，只测量请去掉 --closed 直接 --dry-run")

    if args.dry_run or args.measure:
        servo = Servo(None, args.source, mount, detector=detector, config=config)
        try:
            if targets:
                servo.targets.update({int(cell): np.asarray(xy, dtype=float)
                                      for cell, xy in targets.items()})
            if args.measure:
                found = servo.measure_targets()
                print(json.dumps({str(cell): [round(float(x), 1) for x in xy]
                                  for cell, xy in sorted(found.items())},
                                 ensure_ascii=False))
            else:
                tip, world, view = servo.observe()
                print(json.dumps({
                    "tip_px": [round(float(v), 1) for v in tip.px],
                    "tip_mm": [round(float(v), 2) for v in world],
                    "rms_px": round(view.rms_px, 2),
                    "targets_mm": {str(cell): [round(float(v), 1)
                                               for v in servo.targets.get(cell, target_xy(cell))]
                                   for cell in args.cells},
                }, ensure_ascii=False))
        finally:
            servo.close()
        return

    if args.closed:
        placements = place_closed_cells(args.cells, start=tuple(args.start), source=args.source,
                                        mount=mount, port=args.port, detector=detector,
                                        config=closed_config, targets=targets,
                                        release=args.release)
    else:
        placements = place_cells(args.cells, start=tuple(args.start) if args.start else None,
                                 source=args.source, mount=mount, port=args.port,
                                 detector=detector, config=config, targets=targets,
                                 release=args.release)
    for placement in placements:
        print(json.dumps(asdict(placement), ensure_ascii=False))


if __name__ == "__main__":
    main()