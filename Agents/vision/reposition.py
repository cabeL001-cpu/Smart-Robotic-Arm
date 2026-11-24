"""车体重定位：用顶摄的四个印刷角标把车微调回标定 0/3/6/7/8 时的位姿。

参考像素取自 11-23 标定照片 cell6_result.jpg（1280×720，四个 printed5 中心，
写在 board_reference.json）。相机随车，所以角标在画面里的位置就代表车相对棋盘
的位置和朝向。分别前进、左移、原地左转各探一次，实测每毫秒的角标像素位移作为
3 列雅可比，按 J·Δt = 参考 − 当前 最小二乘求各轴修正毫秒；每轮三个轴按序走完
再统一停车校验，叠加阻尼后才下发，迭代到平移 RMS 与转角都进容差。看不到足够
角标、观测与预测不符或避障拦车就立即停车，不再盲动；回执超时先补发 STOP，
自开的蓝牙链路会重开并重新握手，再按当前视觉重测后续跑。臂不动，只挪车。
"""

import argparse
from contextlib import nullcontext, suppress
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from time import monotonic

import cv2
import numpy as np

from .camera import Mount
from .servo import Servo
from .tags import Detector, describe

if "." in __package__:
    from ..move import avoidance, move as chassis_move
    from ..uno import DEFAULT_PORT, MotionBlocked, ReconnectLink, stop_wheels
else:
    from move import avoidance, move as chassis_move
    from uno import DEFAULT_PORT, MotionBlocked, ReconnectLink, stop_wheels

DEFAULT_REFERENCE = Path(__file__).with_name("board_reference.json")
# 三个轴用正方向探测：前进、左移、原地左转；修正为负时改用反向动作。
AXIS_ACTIONS = (("go", "back"), ("left", "right"), ("rotate-left", "rotate-right"))


class RepositionAborted(RuntimeError):
    """观测或底盘链路异常，已停车，不再继续动作。"""


class LinkLost(RuntimeError):
    """底盘指令没收到回执，已发 STOP；位置交给下一帧视觉重新测量。"""


@dataclass(frozen=True)
class Reference:
    tags: dict                    # printed5 编号 → 参考像素中心
    image_size: tuple = (1280, 720)
    note: str = ""

    @classmethod
    def load(cls, path=DEFAULT_REFERENCE):
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        tags = {int(key): tuple(float(v) for v in xy) for key, xy in data["tags"].items()}
        if len(tags) < 2:
            raise ValueError("参考位姿至少需要两个角标")
        return cls(tags, tuple(data.get("image_size", (1280, 720))), data.get("note", ""))


@dataclass(frozen=True)
class RepositionConfig:
    probe_ms: int = 200              # 测雅可比的单次底盘动作时长
    gain: float = 0.8                # 单步修正阻尼，小于 1 防越过目标
    max_step_ms: int = 300           # 单次修正时长上限，前进/平移/转向共用
    min_step_ms: int = 8             # 低于此时长不再动作，避免在回差内来回
    tolerance_px: float = 4.0        # 角标平移 RMS，约 1.8 mm
    tolerance_deg: float = 1.0
    settle_s: float = 0.6            # 停车后丢弃旧帧的时长
    max_steps: int = 16
    refresh_steps: int = 5           # 每隔几步重测雅可比
    retries: int = 10                # 找角标时连续丢弃的帧数
    min_tags: int = 2                # 同时看到几个参考角标才允许动作
    max_start_px: float = 150.0      # 初始平移偏差上限，超过先人工把车开近
    link_recoveries: int = 2         # 回执超时后重开蓝牙链路的次数
    save_frame: str = ""             # 失败时把最后一帧写到该路径，便于核对视野


@dataclass(frozen=True)
class RepositionResult:
    ok: bool
    rms_px: float          # 末次角标平移误差
    rotation_deg: float    # 末次转角估计
    moves: int             # 下发的底盘动作次数，含雅可比探测
    tags: tuple            # 本次用到的角标编号
    note: str = ""


def _similarity(current, reference):
    """复数最小二乘拟合 current→reference 的相似变换，返回转角、缩放和形状残差。"""
    p = np.asarray(current, dtype=float) - np.mean(current, axis=0)
    q = np.asarray(reference, dtype=float) - np.mean(reference, axis=0)
    zp = p[:, 0] + 1j * p[:, 1]
    zq = q[:, 0] + 1j * q[:, 1]
    denom = float(np.sum(np.abs(zp) ** 2))
    if denom <= 1e-9:
        return 0.0, 1.0, float("inf")
    a = np.sum(zq * np.conj(zp)) / denom
    residual = zq - a * zp
    return (math.degrees(np.angle(a)), float(abs(a)),
            float(np.sqrt(np.mean(np.abs(residual) ** 2))))


class Repositioner:
    """复用 Servo 的相机流与 Uno 连接，只把修正量换成底盘毫秒数。"""

    def __init__(self, servo, reference, *, config=RepositionConfig(), log=print):
        if len(reference.tags) < config.min_tags:
            raise ValueError("参考位姿的角标数少于 min_tags")
        self.servo = servo
        self.reference = reference
        self.config = config
        self.log = log
        self._recoveries = 0

    def _observe(self):
        last = "没读到印刷角标"
        last_tags, last_frame = {}, None
        for _ in range(self.config.retries):
            try:
                _, frame = next(self.servo.stream)
            except StopIteration:
                raise RepositionAborted("图像流结束，重定位需要连续帧") from None
            height, width = frame.shape[:2]
            ref_w, ref_h = tuple(self.reference.image_size)
            if (width, height) != (ref_w, ref_h):
                raise RepositionAborted(
                    f"画面 {width}×{height} 与参考 {ref_w}×{ref_h} 不一致，"
                    "检查分辨率与去畸变设置，或按当前相机重新记录参考")
            tags = self.servo.detector.detect(frame)
            last_tags, last_frame = tags, frame
            centers = {ident: np.asarray(quad, dtype=float).mean(axis=0)
                       for (family, ident), quad in tags.items()
                       if family == "printed5" and ident in self.reference.tags
                       and len(quad) == 4}
            if len(centers) >= self.config.min_tags:
                return centers
            last = f"本帧只认出 {len(centers)} 个参考角标"
        if self.config.save_frame and last_frame is not None:
            cv2.imwrite(self.config.save_frame, last_frame)
            last += f"，最后一帧已存到 {self.config.save_frame}"
        raise RepositionAborted(
            f"连续 {self.config.retries} 帧看不到至少 {self.config.min_tags} 个棋盘角标"
            f"（{last}；最近一帧检测到 {describe(last_tags)}）；"
            "确认棋盘在顶摄视野内且与参考位姿接近")

    def _ids(self, centers):
        return sorted(ident for ident in centers if ident in self.reference.tags)

    def _metrics(self, centers, ids):
        error = np.array([self.reference.tags[i] - centers[i] for i in ids])
        rms = float(np.sqrt(np.mean(np.sum(error ** 2, axis=1))))
        angle, _, _ = _similarity([centers[i] for i in ids],
                                  [self.reference.tags[i] for i in ids])
        return rms, angle

    def _check_start(self, centers):
        ids = self._ids(centers)
        rms, _ = self._metrics(centers, ids)
        if len(ids) >= 3:
            angle, scale, fit = _similarity([centers[i] for i in ids],
                                            [self.reference.tags[i] for i in ids])
            if abs(angle) > 60 or not 0.6 <= scale <= 1.7 or fit > 40:
                raise RepositionAborted(
                    f"当前画面与参考位姿相差太大（转角 {angle:+.0f}°、缩放 {scale:.2f}、"
                    f"形状残差 {fit:.0f} px），先把车开到标定位附近再微调")
            self.log(f"起始偏差：转角 {angle:+.1f}°，缩放 {scale:.3f}，"
                     f"角标形状残差 {fit:.1f} px")
        if rms > self.config.max_start_px:
            raise RepositionAborted(
                f"当前与参考平移相差 {rms:.0f} px（上限 {self.config.max_start_px:.0f} px），"
                "属于搬车而不是微调：先把车开回标定位附近再跑")

    def _drive(self, axis, duration_ms, *, settle=True):
        action = AXIS_ACTIONS[axis][0] if duration_ms > 0 else AXIS_ACTIONS[axis][1]
        try:
            chassis_move(action, abs(duration_ms), link=self.servo.link)
        except MotionBlocked:
            raise
        except (TimeoutError, OSError) as exc:
            with suppress(OSError, TimeoutError):
                stop_wheels(self.servo.link, monotonic() + 1.5)
            raise LinkLost(f"底盘指令没有回执（{exc}），已发 STOP") from None
        if not settle:
            return
        deadline = monotonic() + self.config.settle_s
        for stamp, _ in self.servo.stream:
            if stamp >= deadline:
                return

    def _recover_link(self):
        reopen = getattr(self.servo.link, "reopen", None)
        if reopen is None:
            raise OSError("当前连接不是可重开的蓝牙链路，已停车")
        reopen()
        avoidance(False, link=self.servo.link)

    def _probe(self, centers):
        """分别前进、左移、左转一次，用角标像素位移测出三列雅可比。"""
        deltas = []
        for axis, (action, _) in enumerate(AXIS_ACTIONS):
            before = centers
            self._drive(axis, self.config.probe_ms)
            centers = self._observe()
            shared = [i for i in self._ids(before) if i in centers]
            if len(shared) < self.config.min_tags:
                raise RepositionAborted("探测后丢失参考角标，已停车")
            moved = {i: centers[i] - before[i] for i in shared}
            shift = float(np.mean([np.linalg.norm(d) for d in moved.values()]))
            if shift < 2.0:
                raise RepositionAborted(
                    f"{action} {self.config.probe_ms} ms 后角标几乎没动，"
                    "检查车轮、电池和蓝牙链路")
            deltas.append(moved)
            self.log(f"探测 {action} {self.config.probe_ms} ms → 角标平均位移 {shift:.1f} px")
        shared = [i for i in self._ids(centers) if all(i in delta for delta in deltas)]
        if len(shared) < self.config.min_tags:
            raise RepositionAborted("探测过程中丢失参考角标，已停车")
        jacobian = np.column_stack(
            [np.concatenate([moved[i] for i in shared]) / self.config.probe_ms
             for moved in deltas])
        singular = np.linalg.svd(jacobian, compute_uv=False)
        if singular[-1] < 0.01 * singular[0]:
            raise RepositionAborted("前进、平移和转向在画面里几乎同向，先摆正车头或调整距离")
        return jacobian, shared, centers

    def _observe_step(self, before, predicted):
        """走一步后校验角标位移：与雅可比预测差太多就按误检重试。"""
        want_all = float(np.linalg.norm(np.concatenate(list(predicted.values()))))
        limit = max(12.0, 2.0 * want_all)
        for _ in range(3):
            after = self._observe()
            ids = [i for i in predicted if i in after]
            if len(ids) < self.config.min_tags:
                self.log("动作后丢失参考角标，重试")
                continue
            moved = np.concatenate([after[i] - before[i] for i in ids])
            want = np.concatenate([predicted[i] for i in ids])
            if np.linalg.norm(moved - want) <= limit:
                return after
            self.log(f"角标位移 {np.linalg.norm(moved):.1f} px 与预测 "
                     f"{np.linalg.norm(want):.1f} px 不符，重试")
        raise RepositionAborted("车体动作后角标位移与预测不符，疑似误检，已停车")

    def run(self):
        moves = 0
        try:
            centers = self._observe()
            self._check_start(centers)
            jacobian, shared = None, []
            for step in range(self.config.max_steps + 1):
                ids = [i for i in shared if i in centers] or self._ids(centers)
                if len(ids) < self.config.min_tags:
                    raise RepositionAborted("参考角标不足，无法继续修正")
                rms, angle = self._metrics(centers, ids)
                if rms <= self.config.tolerance_px and abs(angle) <= self.config.tolerance_deg:
                    self.log(f"重定位完成：角标 RMS {rms:.2f} px，转角 {angle:+.2f}°")
                    return RepositionResult(True, rms, angle, moves, tuple(ids))
                if step == self.config.max_steps:
                    break
                if jacobian is None or step % self.config.refresh_steps == 0:
                    jacobian, shared, centers = self._probe(centers)
                    moves += 3
                    continue
                positions = {ident: k for k, ident in enumerate(shared)}
                use = [i for i in shared if i in centers]
                if len(use) < self.config.min_tags:
                    jacobian = None
                    continue
                rows = [2 * positions[i] + offset for i in use for offset in (0, 1)]
                error = np.array([self.reference.tags[i] - centers[i] for i in use]).ravel()
                step_ms = np.clip(
                    np.linalg.lstsq(jacobian[rows], error, rcond=None)[0] * self.config.gain,
                    -self.config.max_step_ms, self.config.max_step_ms)
                durations = [int(round(value)) for value in step_ms]
                active = [k for k, value in enumerate(durations)
                          if abs(value) >= self.config.min_step_ms]
                if not active:
                    # 低于最小步长已进车体回差，继续下发只会来回蹭；
                    # 实际残差若仍明显小于放臂容差，就按当前偏差收尾并写明实测值。
                    done = (rms <= 2 * self.config.tolerance_px
                            and abs(angle) <= 2 * self.config.tolerance_deg)
                    note = (f"修正量低于 {self.config.min_step_ms} ms 的回差，"
                            f"按当前偏差收尾：RMS {rms:.1f} px，转角 {angle:+.2f}°")
                    self.log(("重定位完成：" if done else "重定位中止：") + note)
                    return RepositionResult(done, rms, angle, moves, tuple(use), note)
                predicted = {}
                for ident in use:
                    moved = np.zeros(2)
                    for axis in active:
                        moved += (jacobian[2 * positions[ident]:2 * positions[ident] + 2, axis]
                                  * durations[axis])
                    predicted[ident] = moved
                for count, axis in enumerate(active):
                    self._drive(axis, durations[axis], settle=count == len(active) - 1)
                moves += len(active)
                centers = self._observe_step(centers, predicted)
                rms, angle = self._metrics(centers, [i for i in use if i in centers])
                labels = "，".join(
                    f"{AXIS_ACTIONS[k][0] if durations[k] > 0 else AXIS_ACTIONS[k][1]} "
                    f"{abs(durations[k])} ms" for k in active)
                self.log(f"车体 {labels} → 角标 RMS {rms:.1f} px，转角 {angle:+.2f}°")
        except LinkLost as exc:
            if self._recoveries >= self.config.link_recoveries:
                note = f"{exc}；重连次数用完，停止重定位"
                self.log("重定位中止：" + note)
                return RepositionResult(False, float("nan"), float("nan"), moves, (), note)
            self._recoveries += 1
            self.log(f"{exc}，第 {self._recoveries} 次重连蓝牙后重新测量")
            try:
                self._recover_link()
            except (OSError, TimeoutError) as err:
                note = f"{exc}；蓝牙重连失败：{err}"
                self.log("重定位中止：" + note)
                return RepositionResult(False, float("nan"), float("nan"), moves, (), note)
            return self.run()
        except RepositionAborted as exc:
            self.log(f"重定位中止：{exc}")
            return RepositionResult(False, float("nan"), float("nan"), moves, (), str(exc))
        except MotionBlocked as exc:
            self.log("重定位中止：超声避障拦车，已停车")
            return RepositionResult(False, float("nan"), float("nan"), moves, (),
                                    f"超声避障拦车，已停车：{exc}")
        return RepositionResult(False, rms, angle, moves, tuple(ids),
                                "迭代次数用完仍未回到参考位姿")

    def measure(self):
        """只测当前角标与参考的偏差，不动车。"""
        centers = self._observe()
        ids = self._ids(centers)
        rms, angle = self._metrics(centers, ids)
        return {"tags": ids,
                "centers_px": {str(i): [round(float(v), 1) for v in centers[i]] for i in ids},
                "rms_px": round(rms, 2), "rotation_deg": round(angle, 2)}


def reposition(*, source, mount=None, port=DEFAULT_PORT, link=None, detector=None,
               reference=None, config=RepositionConfig(), log=print):
    """把车微调回参考位姿。全程关避障，结束时恢复（失败重连再试一次）；臂不动。"""
    reference = reference or Reference.load()
    with (nullcontext(link) if link is not None else ReconnectLink(port)) as connection:
        servo = Servo(connection, source, mount,
                      detector=detector or Detector(("36h11", "36h10")), log=log)
        try:
            # 贴盘微调时四面超声几乎必然误拦，动作期间先关掉。
            try:
                avoidance(False, link=connection)
            except (OSError, TimeoutError) as exc:
                log(f"重定位中止：关闭避障没有回执（{exc}）")
                with suppress(OSError, TimeoutError):
                    avoidance(True, link=connection)
                return RepositionResult(False, float("nan"), float("nan"), 0, (),
                                        f"关闭避障没有回执：{exc}")
            log("重定位期间关闭避障，结束后恢复")
            try:
                return Repositioner(servo, reference, config=config, log=log).run()
            finally:
                restored, last = False, None
                for attempt in range(2):
                    try:
                        avoidance(True, link=connection)
                        restored = True
                        break
                    except (OSError, TimeoutError) as exc:
                        last = exc
                        reopen = getattr(connection, "reopen", None)
                        if attempt or reopen is None:
                            break
                        try:
                            reopen()
                        except (OSError, TimeoutError) as err:
                            last = err
                            break
                if not restored:
                    log(f"避障恢复失败（{last}），请手动发送 AVOID ON")
        finally:
            servo.close()


def main():
    p = argparse.ArgumentParser(description="用顶摄四角把车微调回标定 0/3/6/7/8 时的位姿")
    p.add_argument("--source", default="0")
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--config", help="安装参数 JSON，缺省按 1280×720 顶摄")
    p.add_argument("--reference", help="参考位姿 JSON，缺省 vision/board_reference.json")
    p.add_argument("--probe", type=int, default=200, help="测雅可比的底盘动作时长 ms")
    p.add_argument("--gain", type=float, default=0.8, help="单步修正阻尼，小于 1 防越过")
    p.add_argument("--max-step", type=int, default=300, help="单次修正时长上限 ms")
    p.add_argument("--tolerance", type=float, default=4.0, help="角标平移 RMS 到位误差 px")
    p.add_argument("--tolerance-deg", type=float, default=1.0, help="转角到位误差 度")
    p.add_argument("--max-steps", type=int, default=16, help="迭代次数上限")
    p.add_argument("--raw", action="store_true", help="相机用未压缩 YUYV；MJPG 会让小角标读不出")
    p.add_argument("--dry-run", action="store_true", help="只测当前偏差，不动车")
    p.add_argument("--save-frame", help="找不到角标时把最后一帧写到该路径，便于核对视野")
    args = p.parse_args()

    mount = Mount.load(args.config)
    if args.raw:
        mount.raw = True
    if not mount.image_size:
        mount.image_size = (1280, 720)
    reference = Reference.load(args.reference) if args.reference else Reference.load()
    config = RepositionConfig(probe_ms=args.probe, gain=args.gain,
                              max_step_ms=args.max_step, tolerance_px=args.tolerance,
                              tolerance_deg=args.tolerance_deg, max_steps=args.max_steps,
                              save_frame=args.save_frame or "")

    if args.dry_run:
        servo = Servo(None, args.source, mount, detector=Detector(("36h11", "36h10")),
                      log=print)
        try:
            print(json.dumps(Repositioner(servo, reference, config=config).measure(),
                             ensure_ascii=False))
        finally:
            servo.close()
        return

    result = reposition(source=args.source, mount=mount, port=args.port, reference=reference,
                        config=config)
    print(json.dumps(asdict(result), ensure_ascii=False))


if __name__ == "__main__":
    main()