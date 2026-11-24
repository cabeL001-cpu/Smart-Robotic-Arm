"""麦轮底盘启动零点导航：惯性粗定位、整圈墙标测量和停车后的视觉精调。"""

import argparse
from contextlib import contextmanager, suppress
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
from threading import Lock
from time import monotonic

from . import DEFAULT_PORT, move, open_uno, stop_wheels, wait_reply
from .board_route import BoardApproach, BoardRoute
from .imu import rotate, wrap
from .nav_config import NavigationConfig, Origin


def _command(link, verb):
    line = ("NAV " + verb).encode("ascii")
    link.write(line + b"\n")
    wait_reply(link, b"OK " + line)


def reset_navigation_lock(*, uno_port=None):
    """人工排除障碍后解除 Uno 锁存，只停车与解锁，不启动或恢复任何任务。"""
    with open_uno(uno_port or DEFAULT_PORT) as link:
        _command(link, "RESET")


@dataclass(frozen=True)
class Arrival:
    goal_field_m: tuple
    field_x_m: float
    field_y_m: float
    field_yaw_rad: float
    start_x_m: float
    start_y_m: float
    error_m: float
    tag_ids: tuple


@dataclass(frozen=True)
class _Sight:
    transform: Origin
    turn: float
    ids: tuple
    rms: float


class _Progress:
    """用累计改善量判断停滞，允许单个脉冲小于传感器噪声。"""
    def __init__(self, gain, limit):
        self.best, self.stalled = math.inf, 0
        self.gain, self.limit = gain, limit

    def update(self, remaining):
        if remaining < self.best - self.gain:
            self.best, self.stalled = remaining, 0
        else:
            self.stalled += 1
            if self.stalled >= self.limit:
                raise RuntimeError("连续点动无有效进展，已停止导航，请检查方向、速率和定位")


class Navigator:
    """由 open_navigation 创建。一个会话保留一个启动原点，依次执行多个目标。"""
    def __init__(self, config, field, imu, camera, localizer, link):
        self.config, self.field = config, field
        self.control, self.origin = config.control, config.origin
        self.imu, self.camera, self.localizer, self.link = imu, camera, localizer, link
        self.last_frame = self.settled_at = monotonic()
        self.halted = False
        self._task = Lock()
        self.board = (BoardRoute(config.board_approach, field.bounds_m,
                                 field.board_origin_mm, field.board_yaw_deg)
                      if config.board_approach is not None else None)
        self._parked_at_board = False
        self._route_goal = None
        self._route_points = []

    def _check(self, deadline):
        if self.halted:
            raise RuntimeError("导航会话已终止")
        if monotonic() >= deadline:
            raise TimeoutError("本次导航超时")
        self.camera.check_live()
        self.imu.snapshot()
        _command(self.link, "CHECK")

    @contextmanager
    def _operation(self):
        if not self._task.acquire(blocking=False):
            raise RuntimeError("同一导航会话一次只能执行一个目标")
        try:
            if self.halted:
                raise RuntimeError("此会话已终止，请人工确认停车原因后重新建系")
            try:
                yield
            except BaseException:
                self.halted = True
                with suppress(OSError):
                    stop_wheels(self.link)
                raise
        finally:
            self._task.release()

    @contextmanager
    def shared_link(self):
        """在两次导航之间独占复用串口，例如停车后发送关节指令。"""
        with self._operation():
            self._check(monotonic() + self.control.vision_timeout_s)
            yield self.link
            self._check(monotonic() + self.control.vision_timeout_s)

    def _finish(self):
        self.halted = True
        with self._task:
            stop_wheels(self.link)
            _command(self.link, "END")

    def _pulse(self, action, amount, deadline, *, limit_ms=None, reference=None):
        self._check(deadline)
        c = self.control
        ms = round(1000 * amount / self.config.speeds[action])
        ms = min(c.max_pulse_ms, max(c.min_pulse_ms, ms))
        if limit_ms is not None:
            ms = min(ms, limit_ms)
        if self.board is not None and action.startswith("rotate"):
            origin = reference or self.origin
            if origin is not None:
                state, _ = self.imu.snapshot()
                if not self.board.travel.contains(origin.to_field(state.x_m, state.y_m)):
                    raise RuntimeError("离板边或围挡过近，无法在当前位置原地旋转")
        if monotonic() + ms / 1000 + c.settle_s >= deadline:
            raise TimeoutError("剩余时间不足以完成点动和停车")
        self.imu.moving()
        move(action, ms, link=self.link, guarded=True)
        self.settled_at = self.imu.settle(c.settle_s, deadline)
        self._check(deadline)

    def _inside(self, x, y):
        x0, y0, x1, y1 = self.field.bounds_m
        return x0 <= x <= x1 and y0 <= y <= y1

    def _observe(self, deadline):
        # 持续取流但只使用上次停车沉降之后的新帧。检测延迟由 IMU 历史对齐。
        stamp, frame = self.camera.after(max(self.last_frame, self.settled_at), deadline)
        self.last_frame = stamp
        fix = self.localizer.measure(frame)
        if fix is None or not self._inside(fix.x_m, fix.y_m):
            return None
        state = self.imu.at(stamp, deadline)
        return stamp, fix, state

    def _agree(self, x1, y1, a1, x2, y2, a2):
        return (math.hypot(x1-x2, y1-y2) <= self.control.agreement_m
                and abs(wrap(a1-a2)) <= math.radians(self.control.agreement_deg))

    def _matches_reference(self, reference, fix, state):
        x, y = reference.to_field(state.x_m, state.y_m)
        return self._agree(x, y, state.yaw_rad + reference.yaw_rad, fix.x_m, fix.y_m, fix.yaw_rad)

    def _survey_consensus(self, sights):
        # 各墙标是在不同朝向看到的。先用曝光时的 IMU 位姿把它们传到同一时刻，
        # 再比较位置与航向，而不是平均相差约一整圈的原始朝向。
        now, _ = self.imu.snapshot()
        predictions = [(*s.transform.to_field(now.x_m, now.y_m),
                        wrap(now.yaw_rad + s.transform.yaw_rad)) for s in sights]
        groups = [[i for i, other in enumerate(predictions) if self._agree(*pose, *other)]
                  for pose in predictions]
        group = max(groups, key=len, default=[])
        ids = {ident for i in group for ident in sights[i].ids}
        if (len(group) < self.control.confirm_frames or len(group)*2 <= len(sights)
                or len(ids) < self.control.min_scan_tags):
            raise RuntimeError("已完成一圈，墙标观测数量或几何一致性不足，保持停车")
        poses = [predictions[i] for i in group]
        x = sum(p[0] for p in poses) / len(poses)
        y = sum(p[1] for p in poses) / len(poses)
        yaw = math.atan2(sum(math.sin(p[2]) for p in poses), sum(math.cos(p[2]) for p in poses))
        best = min((sights[i] for i in group), key=lambda s: s.rms)
        return Origin.match(x, y, yaw, now), best.turn

    def _face(self, target_turn, reference, deadline):
        progress = _Progress(.4, self.control.stall_pulses)
        while True:
            _, turn = self.imu.snapshot()
            error = math.degrees(wrap(target_turn - turn))
            if abs(error) <= 4.:
                return
            if abs(error) <= self.control.scan_step_deg:
                # 短脉冲有最小转角。墙标已重新入镜且与整圈解一致时，可直接停车精调。
                seen = self._observe(min(deadline, monotonic() + self.control.vision_timeout_s))
                if seen is not None and self._matches_reference(reference, seen[1], seen[2]):
                    return
            progress.update(abs(error))
            self._pulse("rotate-left" if error > 0 else "rotate-right",
                        min(abs(error), self.control.scan_step_deg), deadline,
                        reference=reference)

    def _survey(self, deadline):
        _, start_turn = self.imu.snapshot()
        sights = []
        scan_reference = None
        progress = _Progress(.5, self.control.stall_pulses)
        while True:
            self._check(deadline)
            seen = self._observe(min(deadline, monotonic() + self.control.vision_timeout_s))
            _, turn = self.imu.snapshot()
            if seen is not None:
                _, fix, state = seen
                transform = Origin.match(fix.x_m, fix.y_m, fix.yaw_rad, state)
                sights.append(_Sight(transform, turn, fix.tag_ids, fix.rms_px))
                if scan_reference is None and self.origin is None:
                    scan_reference = transform
            remaining = 360. - math.degrees(turn - start_turn)
            if remaining <= 0:
                break
            progress.update(remaining)
            self._pulse("rotate-left", min(remaining, self.control.scan_step_deg), deadline,
                        reference=scan_reference)
        reference, best_turn = self._survey_consensus(sights)
        self._face(best_turn, reference, deadline)
        return reference

    def _fix_at_rest(self, reference, deadline, *, observe=None):
        """连续独立新帧与扫描结果相符才修正。在停车期间失去参照会超时退出。"""
        until = min(deadline, monotonic() + self.control.vision_timeout_s)
        observe = observe or self._observe
        streak = []
        while len(streak) < self.control.confirm_frames:
            self._check(until)
            seen = observe(until)
            if seen is None:
                streak.clear()
                continue
            stamp, fix, state = seen
            if not self._matches_reference(reference, fix, state):
                streak.clear()
                continue
            if streak and (stamp - streak[-1][0] > .5 or any(
                    not self._agree(f.x_m, f.y_m, f.yaw_rad, fix.x_m, fix.y_m, fix.yaw_rad)
                    for _, f in streak)):
                streak.clear()
            streak.append((stamp, fix))
        x, y = self.origin.to_start(fix.x_m, fix.y_m)
        self.imu.correct(stamp, x, y, wrap(fix.yaw_rad - self.origin.yaw_rad))
        return [f for _, f in streak]

    def _translate(self, target, gain, deadline, *, area=None, max_step=None):
        state, _ = self.imu.snapshot()
        position = self.origin.to_field(state.x_m, state.y_m)
        if not self._inside(*position):
            raise RuntimeError("估计车心已超出场地范围，停止平移")
        ahead, left = rotate(target[0] - state.x_m, target[1] - state.y_m, -state.yaw_rad)
        components = ((ahead, "go", "back", 0.), (left, "left", "right", math.pi / 2))
        for component, positive, negative, direction in sorted(
                components, key=lambda item: abs(item[0]), reverse=True):
            action = positive if component > 0 else negative
            amount = gain * abs(component)
            if area is None:
                self._pulse(action, amount, deadline)
                return
            if abs(component) < 1e-6:
                continue
            speed = self.config.speeds[action]
            distance = self.control.max_pulse_ms * speed / 1000
            if max_step is not None:
                distance = min(distance, max_step)
            angle = state.yaw_rad + self.origin.yaw_rad + direction
            if component < 0:
                angle += math.pi
            # 点动后仍须能看到当前路点，避免横移绕到障碍矩形的另一侧再反复重规划。
            usable = area.clip(position, (math.cos(angle), math.sin(angle)),
                               distance, self.origin.to_field(*target))
            limit_ms = math.floor(usable * 1000 / speed + 1e-9)
            if limit_ms >= self.control.min_pulse_ms:
                self._pulse(action, amount, deadline, limit_ms=limit_ms)
                return
        raise RuntimeError("剩余净空不足以执行最小平移点动")

    def _travel_target(self, target):
        if self.board is None:
            return target, None
        state, _ = self.imu.snapshot()
        position = self.origin.to_field(state.x_m, state.y_m)
        goal = self.origin.to_field(*target)
        area = self.board.travel
        points = self._route_points
        if self._route_goal != goal or not points or not area.allows(position, points[0]):
            points = area.path(position, goal, self.control.tolerance_m + .025)
            self._route_goal = goal
            self._route_points = points
        if len(points) > 1 and math.dist(position, points[0]) <= self.control.tolerance_m:
            if area.allows(position, points[1]):
                points.pop(0)
        distance = sum(math.dist(a, b) for a, b in zip([position, *points], points))
        return self.origin.to_start(*points[0]), distance

    def _approach(self, target, deadline):
        progress = _Progress(min(.01, self.control.coarse_m / 5), self.control.stall_pulses)
        while True:
            state, _ = self.imu.snapshot()
            distance = math.hypot(target[0] - state.x_m, target[1] - state.y_m)
            next_target, route_distance = self._travel_target(target)
            if (distance if route_distance is None else route_distance) <= self.control.coarse_m and (
                    self.board is None or math.dist(next_target, target) < 1e-8):
                return
            progress.update(distance if route_distance is None else route_distance)
            self._translate(next_target, .8, deadline, area=self.board.travel if self.board else None)

    def _navigate(self, x, y, frame, deadline):
        if self.origin is None:
            # 任意起点模式：仅在原地扫描，建立启动系到场地系的刚体变换。
            self.origin = self._survey(deadline)
            self._fix_at_rest(self.origin, deadline)
        goal = (x, y) if frame == "field" else self.origin.to_field(x, y)
        if not self._inside(*goal):
            raise ValueError(f"目标 {goal} 超出标定的车心场地范围")
        if self.board is not None and not self.board.travel.contains(goal):
            raise ValueError("目标离板边或围挡过近，无法执行普通坐标导航")
        if self._parked_at_board:
            self._retreat_from_board(deadline)
        target = self.origin.to_start(*goal)
        self._approach(target, deadline)
        reference = self._survey(deadline)
        progress = _Progress(min(.004, self.control.tolerance_m / 5), self.control.stall_pulses)
        while True:
            fixes = self._fix_at_rest(reference, deadline)
            reference = self.origin  # Tracker 已接受绝对校正，此后的预测使用固定的启动原点。
            errors = [math.hypot(goal[0] - f.x_m, goal[1] - f.y_m) for f in fixes]
            if max(errors) <= self.control.tolerance_m:
                if self.board is not None and not all(
                        self.board.travel.contains((f.x_m, f.y_m)) for f in fixes):
                    raise RuntimeError("目标附近的实测位姿没有足够转向净空")
                self._check(deadline)
                last = fixes[-1]
                return Arrival(goal, last.x_m, last.y_m, last.yaw_rad,
                               *self.origin.to_start(last.x_m, last.y_m), errors[-1],
                               tuple(sorted({i for f in fixes for i in f.tag_ids})))
            progress.update(max(errors))
            if errors[-1] > self.control.tolerance_m:
                next_target, _ = self._travel_target(target)
                self._translate(next_target, .55, deadline, area=self.board.travel if self.board else None)

    def go_to(self, x_m, y_m, *, frame="start"):
        """前往米制坐标。start 为启动系，field 为实测墙标地图系。成功返回 Arrival。"""
        if frame not in ("start", "field") or not all(map(math.isfinite, (x_m, y_m))):
            raise ValueError("坐标须为有限米数，frame 应为 start 或 field")
        if frame == "field" and not self._inside(x_m, y_m):
            raise ValueError("目标超出标定的车心场地范围")
        if frame == "field" and self.board is not None and not self.board.travel.contains((x_m, y_m)):
            raise ValueError("目标离板边或围挡过近，无法执行普通坐标导航")
        with self._operation():
            return self._navigate(x_m, y_m, frame, monotonic() + self.control.timeout_s)

    def go_to_dock(self, name):
        name = name.upper()
        if name not in self.field.docks_m:
            raise ValueError(f"未标定的停靠点 {name}")
        return self.go_to(*self.field.docks_m[name], frame="field")

    def _board_heading_error(self, yaw_rad):
        return math.degrees(wrap(self.board.heading - yaw_rad))

    def _align_for_board(self, deadline):
        progress = _Progress(.4, self.control.stall_pulses)
        while True:
            state, _ = self.imu.snapshot()
            error = self._board_heading_error(state.yaw_rad + self.origin.yaw_rad)
            if abs(error) <= self.board.settings.heading_tolerance_deg:
                return
            progress.update(abs(error))
            self._pulse("rotate-left" if error > 0 else "rotate-right",
                        min(abs(error), self.control.scan_step_deg), deadline)

    def _board_lane(self, goal, deadline, *, observe=None):
        """预靠点到作业位只平移。见到棋盘后，每步停车用下视位姿修正。"""
        tolerance = min(self.control.tolerance_m, .015)
        target = self.origin.to_start(*goal)
        progress = _Progress(.003, self.control.stall_pulses)
        seen_board = False
        while True:
            self._check(deadline)
            state, _ = self.imu.snapshot()
            distance = math.dist(target, (state.x_m, state.y_m))
            if observe is not None and not seen_board and distance > tolerance:
                until = min(deadline, monotonic() + self.control.vision_timeout_s)
                seen_board = observe(until) is not None
            if observe is not None and (seen_board or distance <= tolerance):
                fixes = self._fix_at_rest(self.origin, deadline, observe=observe)
                seen_board = True
                if all(math.dist(goal, (f.x_m, f.y_m)) <= tolerance and
                       abs(self._board_heading_error(f.yaw_rad)) <= self.board.settings.heading_tolerance_deg
                       for f in fixes):
                    last = fixes[-1]
                    return Arrival(goal, last.x_m, last.y_m, last.yaw_rad,
                                   *self.origin.to_start(last.x_m, last.y_m),
                                   math.dist(goal, (last.x_m, last.y_m)),
                                   tuple(sorted({i for f in fixes for i in f.tag_ids})))
                state, _ = self.imu.snapshot()
                distance = math.dist(target, (state.x_m, state.y_m))
            elif observe is None and distance <= tolerance:
                return
            if abs(self._board_heading_error(state.yaw_rad + self.origin.yaw_rad)) > (
                    self.board.settings.heading_tolerance_deg):
                raise RuntimeError("靠近棋盘时航向偏离，已停车")
            if distance <= tolerance:
                progress.update(distance)
                continue
            progress.update(distance)
            self._translate(target, .7, deadline, area=self.board.near, max_step=.025)

    def _retreat_from_board(self, deadline):
        self._board_lane(self.board.staging, deadline)
        state, _ = self.imu.snapshot()
        if not self.board.travel.contains(self.origin.to_field(state.x_m, state.y_m)):
            raise RuntimeError("退出工位后仍未获得足够的转向净空")
        self._parked_at_board = False

    def go_to_board(self, *, top_config, top_source=None):
        """绕到棋盘南侧，按实测作业位停靠，返回下视相机确认的 Arrival。"""
        if self.board is None or self.board.work is None:
            raise ValueError("请在 nav.json 的 board_approach 填写实测 work_distance_m")
        from .nav_sensors import open_camera
        if "." in __package__:
            from ..vision.board import BOARD_LAYOUT, fit_plane
            from ..vision.camera import Mount
            from ..vision.front import FrontFix
            from ..vision.robot import pose_from_plane
            from ..vision.tags import Detector
        else:
            from vision.board import BOARD_LAYOUT, fit_plane
            from vision.camera import Mount
            from vision.front import FrontFix
            from vision.robot import pose_from_plane
            from vision.tags import Detector

        mount = Mount.load(top_config)
        if not mount.image_size or not mount.robot_center_px or not mount.robot_forward_px:
            raise ValueError("请先标定下视相机的分辨率、车心和车头方向")
        source = top_source if top_source is not None else os.environ.get("TOP_CAM")
        if source is None:
            raise ValueError("请指定 --top-source 或 TOP_CAM")
        layout = {key: points for key, points in self.field.floor_layout().items()
                  if key in BOARD_LAYOUT}
        detector = Detector(["36h11"])
        last_stamp = 0.

        with self._operation(), open_camera(str(source), mount, "下视相机") as camera:
            deadline = monotonic() + self.control.timeout_s
            camera.after(monotonic(), min(deadline, monotonic() + self.control.vision_timeout_s))
            self._navigate(*self.board.staging, "field", deadline)
            self._align_for_board(deadline)

            def observe(until):
                nonlocal last_stamp
                stamp, frame = camera.after(max(last_stamp, self.settled_at), until)
                last_stamp = stamp
                plane = fit_plane(detector.detect(frame), layout)
                pose = pose_from_plane(plane, "floor", mount)
                if pose is None or not self.board.near.contains((pose.x_m, pose.y_m)):
                    return None
                fix = FrontFix(pose.x_m, pose.y_m, pose.yaw_rad,
                               tuple(tag_id for _, tag_id in plane.tags_used), pose.rms_px)
                return stamp, fix, self.imu.at(stamp, until)

            arrival = self._board_lane(self.board.work, deadline, observe=observe)
            self._parked_at_board = True
            return arrival


@contextmanager
def open_navigation(config, *, uno_port=None, imu_port=None, front_source=None):
    """启动静止标定，在同一启动原点下顺序执行目标。config 为导航 JSON 路径。"""
    from .nav_sensors import open_front, open_imu
    if "." in __package__:
        from ..vision.camera import Mount
        from ..vision.field import Field
        from ..vision.front import FrontLocalizer
    else:
        from vision.camera import Mount
        from vision.field import Field
        from vision.front import FrontLocalizer

    cfg = NavigationConfig.load(config)
    mount, field = Mount.load(cfg.front_mount), Field.load(cfg.field)
    localizer = FrontLocalizer(mount, field)
    if len(localizer.layout) < cfg.control.min_scan_tags:
        raise ValueError("已标定的墙标数量少于 min_scan_tags")
    x0, y0, x1, y1 = field.bounds_m
    if not all(map(math.isfinite, field.bounds_m)) or x0 >= x1 or y0 >= y1:
        raise ValueError("bounds_m 应为 [xmin,ymin,xmax,ymax]，单位米")
    for name, (x, y) in field.docks_m.items():
        if not (x0 <= x <= x1 and y0 <= y <= y1):
            raise ValueError(f"停靠点 {name} 超出 bounds_m")
    if cfg.origin and not (x0 <= cfg.origin.x_m <= x1 and y0 <= cfg.origin.y_m <= y1):
        raise ValueError("实测启动位置超出 bounds_m")
    imu_port = imu_port or os.environ.get("IMU_PORT")
    source = front_source if front_source is not None else os.environ.get("FRONT_CAM")
    if not imu_port or source is None:
        raise ValueError("请指定 IMU_PORT 与 FRONT_CAM，或传入 imu_port 与 front_source")
    with open_uno(uno_port or DEFAULT_PORT) as link, open_front(str(source), mount) as camera:
        with open_imu(imu_port, cfg.control.calibration_s, cfg.imu_mount_yaw_deg) as imu:
            camera.after(monotonic(), monotonic() + cfg.control.vision_timeout_s)
            nav = Navigator(cfg, field, imu, camera, localizer, link)
            try:
                _command(link, "BEGIN")
                yield nav
            except BaseException:
                with suppress(OSError):
                    nav._finish()
                raise
            else:
                nav._finish()


def go_to(x_m, y_m, *, config, frame="start", uno_port=None, imu_port=None, front_source=None):
    """单次坐标导航，每次调用都以此时静止的车心重新建系。多目标请共用 open_navigation。"""
    with open_navigation(config, uno_port=uno_port, imu_port=imu_port, front_source=front_source) as nav:
        return nav.go_to(x_m, y_m, frame=frame)


def main():
    p = argparse.ArgumentParser(description="车心坐标导航与人工解锁")
    action = p.add_mutually_exclusive_group(required=True)
    action.add_argument("--dock", nargs="+", help="依次前往地图中的停靠点，例如 A B C D")
    action.add_argument("--xy", type=float, nargs=2, metavar=("X米", "Y米"))
    action.add_argument("--board", action="store_true", help="绕行并停靠棋盘作业位")
    action.add_argument("--reset-lock", action="store_true", help="人工清除障碍后解除锁存，不运动车轮")
    p.add_argument("--frame", choices=("start", "field"), default="start", help="--xy 的坐标系")
    p.add_argument("--config", help="导航配置 JSON")
    p.add_argument("--uno-port")
    p.add_argument("--imu-port")
    p.add_argument("--front-source")
    p.add_argument("--top-config", help="下视相机车心及朝向标定 JSON")
    p.add_argument("--top-source", help="下视相机设备号或 V4L2 设备路径")
    p.add_argument("--dry-run", action="store_true", help="只计算棋盘绕行路径，不连接硬件")
    p.add_argument("--from-field", type=float, nargs=2, metavar=("X米", "Y米"),
                   help="仅路径预览使用，场地系车心起点")
    args = p.parse_args()
    try:
        if args.dry_run:
            if not args.board or args.from_field is None:
                p.error("路径预览需要 --board --dry-run --from-field X Y")
            nav_path = Path(args.config).expanduser() if args.config else None
            data = json.loads(nav_path.read_text(encoding="utf-8")) if nav_path else {}
            field = (json.loads((nav_path.parent / data["field"]).read_text(encoding="utf-8"))
                     if nav_path else {})
            board = BoardRoute(BoardApproach(**(data.get("board_approach") or {})),
                               field.get("bounds_m", (0., 0., 2., 2.)),
                               field.get("board_origin_mm", (900., 900.)),
                               field.get("board_yaw_deg", 0.))
            tolerance = data.get("control", {}).get("tolerance_m", .035)
            route = board.travel.path(tuple(args.from_field), board.staging, tolerance + .025)
            print(json.dumps({"waypoints_field_m": route, "heading_deg": board.settings.heading_deg,
                              "work_field_m": board.work}, allow_nan=False))
            return
        if args.from_field is not None:
            p.error("--from-field 仅用于 --dry-run")
        if args.reset_lock:
            reset_navigation_lock(uno_port=args.uno_port)
            print("已解除导航锁存，车轮保持停车。")
            return
        if not args.config:
            p.error("导航需要 --config")
        if args.board and not args.top_config:
            p.error("--board 需要 --top-config")
        with open_navigation(args.config, uno_port=args.uno_port, imu_port=args.imu_port,
                             front_source=args.front_source) as nav:
            if args.dock:
                for name in args.dock:
                    print(json.dumps(asdict(nav.go_to_dock(name)), allow_nan=False), flush=True)
            elif args.board:
                result = nav.go_to_board(top_config=args.top_config, top_source=args.top_source)
                print(json.dumps(asdict(result), allow_nan=False))
            else:
                print(json.dumps(asdict(nav.go_to(*args.xy, frame=args.frame)), allow_nan=False))
    except (OSError, RuntimeError, ValueError, KeyError) as error:
        p.exit(1, f"导航停止：{error}\n")


if __name__ == "__main__":
    main()
