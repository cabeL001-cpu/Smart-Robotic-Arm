"""南侧路线 + 三处前摄墙标距离记录：东行、左转、北行，并在三处记录到墙标的实测距离。

路线完全复用队友的前摄闭环测距：每走一小段停车量一次到墙标的距离，按起步与当前
距离之差累计位移，走满即停，不再用固定毫秒；航向只用 IMU 陀螺积分。额外的点是在
三个时刻多测一次距离并记录（只记录，不做任何微调或闭环修正）：

  1. 左转前（东行结束）：面前的 id=3 墙标
  2. 左转后（转向完成）：面前的 id=1 墙标
  3. 北行结束：正面前摄拍到的 id=1 墙标

每条记录是一行 JSON，打印到终端并追加写入 --record 指定的文件。标定只需要前摄内参
（image_size + camera_matrix，可选 distortion）和墙标黑框边长 side_mm，不需要安装外参
front_xyz_mm / front_ypr_deg，也不需要墙标高度与朝向。
"""

import argparse
import json
from contextlib import suppress

from . import DEFAULT_PORT, MotionBlocked, ReconnectLink, avoidance, stop_wheels
from .heading import turn_to
from .nav_sensors import open_imu
from .tag_route import TagRanger
from .south_route_tag import drive_leg

if "." in __package__:
    from ..vision.camera import Mount
    from ..vision.field import Field
else:
    from vision.camera import Mount
    from vision.field import Field


EAST_M = .73               # 东行 73 cm
NORTH_M = .435             # 左转后北行 43.5 cm（队友实测后定）
TURN_DEG = 85.5            # 原地左转角度
TAG_BEFORE = 3             # 左转前（车头朝东）面前的墙标
TAG_AFTER = 1              # 左转后（车头朝北）面前的墙标
TAG_END = 1                # 北行结束正面前摄拍到的墙标


def _log(message):
    print(message, flush=True)


def record(moment, ranger, tag_id, out):
    """测一次指定墙标的距离并记为一行 JSON；读不到时记为 detected=False，不中断路线。"""
    ranger.tag_id = tag_id
    fix = ranger.measure()
    if fix is None:
        entry = {"moment": moment, "tag_id": tag_id, "detected": False}
    else:
        entry = {"moment": moment, "tag_id": fix.tag_id, "z_mm": round(fix.z_mm, 1),
                 "x_mm": round(fix.x_mm, 1), "y_mm": round(fix.y_mm, 1),
                 "rms_px": round(fix.rms_px, 2)}
    line = json.dumps(entry, ensure_ascii=False)
    _log(f"[{moment}] {line}")
    if out is not None:
        out.write(line + "\n")
        out.flush()


def main():
    parser = argparse.ArgumentParser(description="南侧路线 + 三处前摄墙标距离记录")
    parser.add_argument("--east-cm", type=float, default=EAST_M * 100, help="东行距离，默认 73 cm")
    parser.add_argument("--north-cm", type=float, default=NORTH_M * 100, help="左转后北行距离，默认 43.5 cm")
    parser.add_argument("--turn-deg", type=float, default=TURN_DEG, help="原地左转角度，默认 85.5°")
    parser.add_argument("--camera", default="0", help="前摄设备、编号或 V4L2 路径")
    parser.add_argument("--config", default="move/calibration/front.json",
                        help="前摄内参 JSON，须含 image_size 与 camera_matrix")
    parser.add_argument("--field", default="move/calibration/field.json", help="场地与墙标尺寸")
    parser.add_argument("--side-mm", type=float, help="墙标黑框边长，默认取场地文件")
    parser.add_argument("--tag-before", type=int, default=TAG_BEFORE, help="左转前记录的前方墙标 id")
    parser.add_argument("--tag-after", type=int, default=TAG_AFTER, help="左转后记录的前方墙标 id")
    parser.add_argument("--tag-end", type=int, default=TAG_END, help="北行结束记录的前方墙标 id")
    parser.add_argument("--record", help="把三行 JSON 追加写入的文件，缺省只打印到终端")
    parser.add_argument("--tolerance-cm", type=float, default=1.5, help="允许的位移偏差")
    parser.add_argument("--chunk-ms", type=int, default=400, help="单次点动上限毫秒数")
    parser.add_argument("--speed", type=float, default=.15, help="起步速度估计，米每秒")
    parser.add_argument("--min-z-cm", type=float, default=35., help="与墙标保持的最小距离")
    parser.add_argument("--deadband-deg", type=float, default=1.5, help="直行航向死区")
    parser.add_argument("--ms-per-deg", type=float, default=45., help="短脉冲转向毫秒每度")
    parser.add_argument("--yaw-scale", type=float, default=1., help="陀螺航向比例修正")
    parser.add_argument("--imu-port", default="/dev/ttyUSB0")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--calibration-s", type=float, default=2., help="IMU 静止标定时间")
    parser.add_argument("--no-avoidance", action="store_true",
                        help="动作期间关闭四向避障，结束后恢复开启")
    parser.add_argument("--dry-run", action="store_true", help="只打印计划，不接硬件")
    args = parser.parse_args()
    if args.east_cm <= 0 or args.north_cm <= 0:
        parser.error("两段距离都必须为正")

    field = Field.load(args.field)
    side_mm = args.side_mm
    if side_mm is None:
        sides = [tag["side_mm"] for tag in (field.wall_tags or [])]
        side_mm = next(iter(sides), 200.) if sides else 200.
    _log(f"计划：东行 {args.east_cm:.1f} cm → 左转 {args.turn_deg:.1f}° → 北行 {args.north_cm:.1f} cm"
         f"（墙标边长 {side_mm:.0f} mm），全部按前摄闭环测距。")
    _log(f"记录墙标：左转前 id={args.tag_before}，左转后 id={args.tag_after}，北行结束 id={args.tag_end}")
    if args.dry_run:
        return

    mount = Mount.load(args.config)
    if not mount.camera_matrix:
        raise SystemExit(f"{args.config} 没有相机内参，请填写或换用实测标定文件。\n")
    ranger = TagRanger(args.camera, mount, tag_id=args.tag_before, side_mm=side_mm,
                       family=field.wall_family)
    out = open(args.record, "a", encoding="utf-8") if args.record else None
    try:
        with open_imu(args.imu_port, args.calibration_s, 0.) as imu, \
                ReconnectLink(args.port) as link:
            _log("IMU 已标定（只用陀螺积分）。")
            if args.no_avoidance:
                avoidance(False, link=link)
                _log("已关闭四向避障，动作结束后恢复。")
            try:
                travelled, final_z = drive_leg(
                    link, imu, ranger, distance_m=args.east_cm / 100,
                    tolerance_m=args.tolerance_cm / 100, speed_mps=args.speed,
                    chunk_ms=(100, args.chunk_ms), deadband_deg=args.deadband_deg,
                    ms_per_deg=args.ms_per_deg, min_z_mm=args.min_z_cm * 10)
                _log(f"东行完成：实测位移 {travelled * 100:.1f} cm，距墙标 {final_z:.0f} mm。")
                record("before_turn", ranger, args.tag_before, out)

                error = turn_to(args.turn_deg, imu=imu, link=link, deadband_deg=1.,
                                fine_ms_per_deg=args.ms_per_deg, yaw_scale=args.yaw_scale)
                _log(f"转向完成，目标 {args.turn_deg:+.1f}°，最终偏差 {error:+.1f}°。")
                record("after_turn", ranger, args.tag_after, out)

                travelled, final_z = drive_leg(
                    link, imu, ranger, distance_m=args.north_cm / 100, target_deg=args.turn_deg,
                    tolerance_m=args.tolerance_cm / 100, speed_mps=args.speed,
                    chunk_ms=(100, args.chunk_ms), deadband_deg=args.deadband_deg,
                    ms_per_deg=args.ms_per_deg, min_z_mm=args.min_z_cm * 10)
                _log(f"北行完成：实测位移 {travelled * 100:.1f} cm，距墙标 {final_z:.0f} mm。")
                record("end_north", ranger, args.tag_end, out)
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
        raise SystemExit(130, "路线已中断。\n")
    finally:
        ranger.close()
        if out is not None:
            out.close()


if __name__ == "__main__":
    main()
