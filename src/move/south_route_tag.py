"""南侧路线：前摄墙标闭环测距东行 73 cm，原地左转 85.5°，北行 43.5 cm。

两段直行的距离都由前摄 PnP 实测：每走一小段停车量一次到墙标的距离，按起步与
当前距离之差累计位移，走满即停；不再用固定毫秒。航向只用 IMU 的陀螺积分
（`heading.correct_heading` 读 `snapshot_gyro`），不用磁力计或 AHRS 航向。
默认参数对应 south_route.py 的场地路线；source 文件均不改动，这里只复用
tag_route 的 TagRanger 和 heading 的转向函数。
"""

import argparse
from contextlib import suppress
from time import monotonic

if "." in __package__:
    from ..vision.camera import Mount
    from ..vision.field import Field
else:
    from vision.camera import Mount
    from vision.field import Field

from . import DEFAULT_PORT, MotionBlocked, ReconnectLink, avoidance, move, stop_wheels
from .heading import correct_heading, turn_to
from .nav_sensors import open_imu
from .tag_route import TagRanger

EAST_M = .73
NORTH_M = .435
TURN_DEG = 85.5


def drive_leg(link, imu, ranger, *, distance_m, target_deg=0., tolerance_m=.015,
              speed_mps=.15, chunk_ms=(100, 400), deadband_deg=1.5,
              ms_per_deg=45., min_z_mm=350., stall_chunks=3, lost_frames=3,
              timeout_s=90.):
    """按前摄测距前进 distance_m，航向保持在 target_deg，返回 (位移 m, 停车距离 mm)。"""
    start = ranger.measure()
    if start is None:
        raise RuntimeError("起步时前摄读不到墙标，请让墙标完整入镜后重试")
    if start.z_mm - distance_m * 1000 < min_z_mm:
        raise RuntimeError(f"当前距离 {start.z_mm:.0f} mm 不足以安全前进 {distance_m * 100:.0f} cm")
    print(f"起步 distance {start.z_mm:.0f} mm（tag {start.tag_id}），"
          f"目标前进 {distance_m * 100:.0f} cm，航向 {target_deg:+.1f}°", flush=True)
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
        error = correct_heading(imu, link, target_deg=target_deg, deadband_deg=deadband_deg,
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
    parser = argparse.ArgumentParser(description="南侧路线：前摄闭环测距东行、左转、北行")
    parser.add_argument("--east-cm", type=float, default=EAST_M * 100,
                        help="东行距离，默认 73 cm")
    parser.add_argument("--north-cm", type=float, default=NORTH_M * 100,
                        help="左转后北行距离，默认 43.5 cm")
    parser.add_argument("--turn-deg", type=float, default=TURN_DEG,
                        help="原地左转角度，默认 85.5°")
    parser.add_argument("--camera", default="0", help="前摄设备、编号或 V4L2 路径")
    parser.add_argument("--config", default="move/calibration/front.json",
                        help="前摄内参 JSON，须含 image_size 与 camera_matrix")
    parser.add_argument("--field", default="move/calibration/field.json", help="场地与墙标尺寸")
    parser.add_argument("--tag", type=int, help="指定墙标 ID；缺省每段自动锁定第一块读到的")
    parser.add_argument("--side-mm", type=float, help="墙标黑框边长，默认取场地文件")
    parser.add_argument("--tolerance-cm", type=float, default=1.5, help="允许的位移偏差")
    parser.add_argument("--chunk-ms", type=int, default=400, help="单次点动上限毫秒数")
    parser.add_argument("--speed", type=float, default=.15, help="起步速度估计，米每秒")
    parser.add_argument("--min-z-cm", type=float, default=35., help="与墙标保持的最小距离")
    parser.add_argument("--skip-east", action="store_true",
                        help="跳过东行，从当前位置直接左转和北行")
    parser.add_argument("--no-turn", action="store_true", help="只走东行，不转向和北行")
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
    if args.east_cm <= 0 or args.north_cm <= 0:
        parser.error("两段距离都必须为正")
    if args.chunk_ms < 100:
        parser.error("--chunk-ms 不能小于 100")
    if args.skip_east and args.no_turn:
        parser.error("--skip-east 与 --no-turn 同时给出时没有动作")

    field = Field.load(args.field)
    side_mm = args.side_mm
    if side_mm is None:
        sides = {tag["id"]: tag["side_mm"] for tag in (field.wall_tags or [])}
        side_mm = sides.get(args.tag) or next(iter(sides.values()), 200.)
    legs = "" if args.skip_east else f"东行 {args.east_cm:.1f} cm"
    if not args.no_turn:
        legs += (" → " if legs else "") + f"左转 {args.turn_deg:.1f}° → 北行 {args.north_cm:.1f} cm"
    print(f"计划：{legs}，全部按前摄墙标闭环测距（边长 {side_mm:.0f} mm）。", flush=True)
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
            print("IMU 已标定（只用陀螺积分）。", flush=True)
            if args.no_avoidance:
                avoidance(False, link=link)
                print("已关闭四向避障，动作结束后恢复。", flush=True)
            try:
                if not args.skip_east:
                    travelled, final_z = drive_leg(
                        link, imu, ranger, distance_m=args.east_cm / 100,
                        tolerance_m=args.tolerance_cm / 100, speed_mps=args.speed,
                        chunk_ms=(100, args.chunk_ms), deadband_deg=args.deadband_deg,
                        ms_per_deg=args.ms_per_deg, min_z_mm=args.min_z_cm * 10)
                    print(f"东行完成：实测位移 {travelled * 100:.1f} cm，"
                          f"距墙标 {final_z:.0f} mm。", flush=True)
                if not args.no_turn:
                    error = turn_to(args.turn_deg, imu=imu, link=link,
                                    deadband_deg=1., fine_ms_per_deg=args.ms_per_deg,
                                    yaw_scale=args.yaw_scale)
                    print(f"转向完成，目标 {args.turn_deg:+.1f}°，最终偏差 {error:+.1f}°。",
                          flush=True)
                    if args.tag is None:
                        ranger.tag_id = None
                    travelled, final_z = drive_leg(
                        link, imu, ranger, distance_m=args.north_cm / 100,
                        target_deg=args.turn_deg,
                        tolerance_m=args.tolerance_cm / 100, speed_mps=args.speed,
                        chunk_ms=(100, args.chunk_ms), deadband_deg=args.deadband_deg,
                        ms_per_deg=args.ms_per_deg, min_z_mm=args.min_z_cm * 10)
                    print(f"北行完成：实测位移 {travelled * 100:.1f} cm，"
                          f"距墙标 {final_z:.0f} mm。", flush=True)
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


if __name__ == "__main__":
    main()