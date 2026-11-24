"""前摄墙标闭环测距直行，到点后按 IMU 原地转向。

直行不靠固定毫秒数：每走一小段停车，用前摄 PnP 解出到墙标平面的距离，
按起步距离与当前距离之差累计位移，走满即停；段间用陀螺把航向拉回起步方向。
转向沿用 heading.turn_to 的 IMU 闭环。
"""

import argparse
from contextlib import suppress
from dataclasses import dataclass
from time import monotonic

import cv2
import numpy as np

if "." in __package__:
    from ..vision.camera import Mount, live_source
    from ..vision.field import Field
    from ..vision.tags import Detector
else:
    from vision.camera import Mount, live_source
    from vision.field import Field
    from vision.tags import Detector

from . import DEFAULT_PORT, MotionBlocked, ReconnectLink, avoidance, move, stop_wheels
from .heading import correct_heading, turn_to
from .nav_sensors import open_imu


@dataclass(frozen=True)
class RangeFix:
    tag_id: int
    z_mm: float
    x_mm: float
    y_mm: float
    rms_px: float


class TagRanger:
    """只读前摄，用 PnP 给出镜头到墙标平面的距离。"""

    def __init__(self, source, mount, *, tag_id=None, side_mm=200., family="36h11"):
        device = int(source) if str(source).isdecimal() else str(source)
        backend = cv2.CAP_V4L2 if live_source(device) else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(device, backend)
        if not self.cap.isOpened():
            raise OSError(f"无法打开前摄 {source}")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        if mount.image_size:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, mount.image_size[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, mount.image_size[1])
        self.K = np.asarray(mount.camera_matrix, dtype=float)
        if self.K.shape != (3, 3):
            raise ValueError("前摄配置缺少 3×3 camera_matrix")
        self.D = np.asarray(mount.distortion, dtype=float) if mount.distortion else np.zeros(5)
        self.family, self.tag_id, self.side_mm = family, tag_id, side_mm
        half = side_mm / 2
        # 与检测器角点顺序一致：左上、右上、右下、左下，码面 Y 向上。
        self.object = np.array([[-half, half, 0.], [half, half, 0.],
                                [half, -half, 0.], [-half, -half, 0.]])
        self.detector = Detector((family,), board=False)
        self.expected = tuple(int(v) for v in mount.image_size) if mount.image_size else ()

    def close(self):
        self.cap.release()

    def _read_tag(self):
        ok, frame = self.cap.read()
        if not ok:
            return None
        if self.expected and frame.shape[1::-1] != self.expected:
            raise ValueError(f"前摄分辨率 {frame.shape[1::-1]} 与标定 {self.expected} 不一致")
        tags = self.detector.detect(frame)
        if not tags:
            return None
        if self.tag_id is None:
            self.tag_id = sorted(ident for fam, ident in tags if fam == self.family)[0]
        corners = tags.get((self.family, self.tag_id))
        if corners is None:
            return None
        ok, rvec, tvec = cv2.solvePnP(self.object, np.asarray(corners, dtype=float),
                                      self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        t = tvec.ravel()
        if t[2] <= 0:
            return None
        projected, _ = cv2.projectPoints(self.object, rvec, tvec, self.K, self.D)
        rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - corners) ** 2, axis=1))))
        return RangeFix(self.tag_id, float(t[2]), float(t[0]), float(t[1]), rms)

    def measure(self, *, samples=5, discard=2, max_rms_px=2.5):
        """丢掉过期帧后连续解算，返回各帧中位数；有效帧不足一半时返回 None。"""
        fixes = []
        for _ in range(samples + discard):
            fix = self._read_tag()
            if fix is not None and fix.rms_px <= max_rms_px:
                fixes.append(fix)
        if len(fixes) * 2 < samples:
            return None
        median = lambda key: float(np.median([getattr(fix, key) for fix in fixes]))
        return RangeFix(self.tag_id, median("z_mm"), median("x_mm"), median("y_mm"),
                        max(fix.rms_px for fix in fixes))


def drive_by_tag(link, imu, ranger, *, distance_m, tolerance_m=.015,
                 speed_mps=.15, chunk_ms=(100, 400), deadband_deg=1.5,
                 ms_per_deg=45., min_z_mm=350., stall_chunks=3, lost_frames=3,
                 timeout_s=90.):
    """按前摄测距前进 distance_m，返回 (累计位移 m, 停车后距离 mm)。"""
    start = ranger.measure()
    if start is None:
        raise RuntimeError("起步时前摄读不到墙标，请让墙标完整入镜后重试")
    if start.z_mm - distance_m * 1000 < min_z_mm:
        raise RuntimeError(f"当前距离 {start.z_mm:.0f} mm 不足以安全前进 {distance_m * 100:.0f} cm")
    print(f"起步 distance {start.z_mm:.0f} mm（tag {start.tag_id}），"
          f"目标前进 {distance_m * 100:.0f} cm", flush=True)
    travelled_m = commanded_s = 0.
    last_z = start.z_mm
    stalls = lost = 0
    deadline = monotonic() + timeout_s
    while distance_m - travelled_m > tolerance_m:
        if monotonic() > deadline:
            raise TimeoutError("前摄闭环直行超时")
        remaining = distance_m - travelled_m
        ms = round(min(chunk_ms[1], max(chunk_ms[0], remaining / max(speed_mps, .06) * 1000)))
        move("go", ms, link=link)
        error = correct_heading(imu, link, target_deg=0., deadband_deg=deadband_deg,
                                ms_per_deg=ms_per_deg, max_pulse_ms=300, attempts=3)
        fix = ranger.measure()
        if fix is None:
            lost += 1
            print(f"  本次测距失败（{lost}/{lost_frames}），残余位移 {remaining * 100:.1f} cm", flush=True)
            if lost >= lost_frames:
                raise RuntimeError("直行中连续读不到墙标，已保持停车")
            continue
        lost = 0
        progress_mm = last_z - fix.z_mm
        travelled_m += progress_mm / 1000
        commanded_s += ms / 1000
        if progress_mm < 3.:
            stalls += 1
            if stalls >= stall_chunks:
                raise RuntimeError("连续点动测距无明显变化，可能被阻挡或方向错误")
        else:
            stalls = 0
        last_z = fix.z_mm
        speed_mps = min(.30, max(.06, travelled_m / max(commanded_s, .001)))
        print(f"  位移 {travelled_m * 100:.1f} cm，距离 {fix.z_mm:.0f} mm，"
              f"段 {ms} ms，航向偏差 {error:+.1f}°", flush=True)
        if fix.z_mm < min_z_mm:
            raise RuntimeError(f"与墙标距离仅 {fix.z_mm:.0f} mm，保持停车")
    return travelled_m, last_z


def main():
    parser = argparse.ArgumentParser(description="前摄墙标闭环测距前进，再原地左转")
    parser.add_argument("--distance-cm", type=float, default=73.,
                        help="前摄闭环前进的距离，默认 73 cm")
    parser.add_argument("--turn-deg", type=float, default=90.,
                        help="前进后原地转向角度，左转为正，默认 90°")
    parser.add_argument("--camera", default="0", help="前摄设备、编号或 V4L2 路径")
    parser.add_argument("--config", default="move/calibration/front.json",
                        help="前摄内参 JSON，须含 image_size 与 camera_matrix")
    parser.add_argument("--field", default="move/calibration/field.json", help="场地与墙标尺寸")
    parser.add_argument("--tag", type=int, help="指定墙标 ID，默认锁定第一块读到的")
    parser.add_argument("--side-mm", type=float, help="墙标黑框边长，默认取场地文件")
    parser.add_argument("--tolerance-cm", type=float, default=1.5, help="允许的位移偏差")
    parser.add_argument("--no-turn", action="store_true", help="只走直行，不转向")
    parser.add_argument("--imu-port", default="/dev/ttyUSB0")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--calibration-s", type=float, default=2., help="IMU 静止标定时间")
    parser.add_argument("--deadband-deg", type=float, default=1.5, help="直行航向死区")
    parser.add_argument("--ms-per-deg", type=float, default=45., help="短脉冲转向毫秒每度")
    parser.add_argument("--yaw-scale", type=float, default=1., help="陀螺航向比例修正")
    parser.add_argument("--no-avoidance", action="store_true",
                        help="动作期间关闭四向避障，结束后恢复开启")
    parser.add_argument("--dry-run", action="store_true", help="只打印计划，不接硬件")
    args = parser.parse_args()
    if args.distance_cm <= 0:
        parser.error("--distance-cm 必须为正")

    field = Field.load(args.field)
    side_mm = args.side_mm
    if side_mm is None:
        sides = {tag["id"]: tag["side_mm"] for tag in (field.wall_tags or [])}
        side_mm = sides.get(args.tag) or next(iter(sides.values()), 200.)
    print(f"计划：前摄闭环前进 {args.distance_cm:.1f} cm（墙标边长 {side_mm:.0f} mm）"
          f"{'' if args.no_turn else f'，随后左转 {args.turn_deg:.1f}°'}。", flush=True)
    if args.dry_run:
        return

    mount = Mount.load(args.config)
    if not mount.camera_matrix:
        raise SystemExit(f"{args.config} 没有相机内参，请填写或换用实测标定文件。\n")
    ranger = TagRanger(args.camera, mount, tag_id=args.tag, side_mm=side_mm,
                       family=field.wall_family)
    try:
        with open_imu(args.imu_port, args.calibration_s, 0.) as imu, \
                ReconnectLink(args.port) as link:
            print("IMU 已标定。", flush=True)
            if args.no_avoidance:
                avoidance(False, link=link)
                print("已关闭四向避障，动作结束后恢复。", flush=True)
            try:
                travelled, final_z = drive_by_tag(
                    link, imu, ranger, distance_m=args.distance_cm / 100,
                    tolerance_m=args.tolerance_cm / 100,
                    deadband_deg=args.deadband_deg, ms_per_deg=args.ms_per_deg)
                print(f"直行完成：实测位移 {travelled * 100:.1f} cm，"
                      f"距墙标 {final_z:.0f} mm。", flush=True)
                if not args.no_turn:
                    error = turn_to(args.turn_deg, imu=imu, link=link,
                                    deadband_deg=1., fine_ms_per_deg=args.ms_per_deg,
                                    yaw_scale=args.yaw_scale)
                    print(f"转向完成，目标 {args.turn_deg:+.1f}°，最终偏差 {error:+.1f}°。", flush=True)
            finally:
                with suppress(OSError):
                    stop_wheels(link)
                if args.no_avoidance:
                    with suppress(OSError):
                        avoidance(True, link=link)
    except MotionBlocked as error:
        raise SystemExit(f"底盘因避障保持停车：{error}\n")
    except (RuntimeError, ValueError, TimeoutError, OSError) as error:
        raise SystemExit(f"动作中止：{error}\n")
    except KeyboardInterrupt:
        raise SystemExit(130, "动作已中断。\n")
    finally:
        ranger.close()


if __name__ == "__main__":
    main()