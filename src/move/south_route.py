"""南侧路线验收：东行 73 cm，原地左转 85.5°，北行 41 cm 贴住九宫格南边。

起点车心 (0.17, 0.17) m、车头朝 +X。九宫格 30 cm 见方，中心 (0.90, 0.90) m，
南边在 y=0.75 m。车身长 34 cm，车头前沿顶到南边时车心 y=0.58 m。
直行沿用 IMU 航向保持的分段修正参数，转向也用 IMU 闭环。

默认时长是 2026-11-23 在实车上逐次修正得到的：东行 4760 ms、北行 2830 ms、
左转 85.5°。换电池、换场地后按实测偏差用命令行参数微调，稳定后再改这里的常数。
"""

import argparse

from . import DEFAULT_PORT, MotionBlocked, ReconnectLink
from .heading import go_straight, turn_to
from .nav_sensors import open_imu


START_M = (.17, .17)        # 车心起点，贴住场地左下角
EAST_M = .73                # 东行到九宫格中线 x=0.90
BOARD_CENTER_M = (.90, .90)
BOARD_SIDE_M = .30
CAR_LENGTH_M = .34
NORTH_M = .41               # 车头前沿不留缝时北行的距离
EAST_MS = 4760              # 实测：东行 73 cm，满电
NORTH_MS = 2830             # 实测：北行 41 cm，车头前沿贴住九宫格南边
TURN_DEG = 85.5             # 实测：物理车头与九宫格南边平行


def _ms(distance_m, speed_mps):
    return round(distance_m / speed_mps * 1000)


def _log(message):
    print(message, flush=True)


def drive_to_board(*, imu, link, east_ms=EAST_MS, north_ms=NORTH_MS, turn_deg=TURN_DEG,
                   chunk_ms=400, start_skip_ms=1600, deadband_deg=1., ms_per_deg=45.,
                   yaw_scale=1., turn_deadband_deg=1., log=_log, on_phase=None):
    """在已打开的 IMU 与 Uno 连接上跑完整条南侧路线，返回最终航向偏差（度）。

    供总控直接调用：串口和 IMU 由调用方打开，路线本身只负责运动。
    on_phase 是可选回调，在每段结束、车已停稳后以阶段名（"before_turn" /
    "after_turn" / "end_north"）调用一次，用于顺带记录；它只读传感器、不发运动，
    默认 None 表示不记录，不影响原有运动流程。
    """
    log("== 东行 ==")
    go_straight(east_ms, imu=imu, link=link, chunk_ms=chunk_ms,
                target_deg=0., skip_correction_ms=start_skip_ms,
                deadband_deg=deadband_deg, ms_per_deg=ms_per_deg,
                yaw_scale=yaw_scale)
    if on_phase:
        on_phase("before_turn")
    log("== 原地左转 ==")
    turn_to(turn_deg, imu=imu, link=link, deadband_deg=turn_deadband_deg,
            fine_ms_per_deg=ms_per_deg, yaw_scale=yaw_scale)
    if on_phase:
        on_phase("after_turn")
    log("== 北行 ==")
    error = go_straight(north_ms, imu=imu, link=link, chunk_ms=chunk_ms,
                        target_deg=turn_deg, deadband_deg=deadband_deg,
                        ms_per_deg=ms_per_deg, yaw_scale=yaw_scale)
    if on_phase:
        on_phase("end_north")
    log(f"南侧路线完成，最终航向偏差 {error:+.1f}°。")
    return error


def main():
    parser = argparse.ArgumentParser(description="南侧路线验收：东行、左转、北行")
    parser.add_argument("--speed", type=float,
                        help="直行速度，米每秒；给出后按几何距离重算两段时长")
    parser.add_argument("--gap-cm", type=float, default=0.,
                        help="车头前沿与九宫格南边的距离，默认 0（贴住）")
    parser.add_argument("--turn-deg", type=float, default=TURN_DEG,
                        help="原地左转角度，默认按实测 85.5° 与九宫格南边对齐")
    parser.add_argument("--east-ms", type=int, help="直接指定东行毫秒数，覆盖默认值")
    parser.add_argument("--north-ms", type=int, help="直接指定北行毫秒数，覆盖默认值")
    parser.add_argument("--chunk-ms", type=int, default=400, help="航向保持的直行分段")
    parser.add_argument("--start-skip-ms", type=int, default=1600,
                        help="东行起步先不修正的毫秒数，等后方离边界超过 30 cm")
    parser.add_argument("--deadband-deg", type=float, default=1., help="直行允许的航向偏差")
    parser.add_argument("--ms-per-deg", type=float, default=45.,
                        help="短脉冲修正的毫秒每度，实测约 40–50")
    parser.add_argument("--yaw-scale", type=float, default=1.,
                        help="陀螺航向比例修正，等于 90 除以手推 90° 时陀螺列读数")
    parser.add_argument("--turn-deadband-deg", type=float, default=1., help="转向允许的偏差")
    parser.add_argument("--imu-port", default="/dev/ttyUSB0")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--calibration-s", type=float, default=2., help="IMU 静止标定时间")
    parser.add_argument("--dry-run", action="store_true", help="只打印路线与理论终点，不动作")
    args = parser.parse_args()
    if args.speed is not None and args.speed <= 0:
        raise SystemExit("直行速度必须为正。\n")

    north_m = (BOARD_CENTER_M[1] - BOARD_SIDE_M / 2 - args.gap_cm / 100
               - CAR_LENGTH_M / 2 - START_M[1])
    if args.speed is not None:
        # 显式给速度时按几何距离估算；默认不走这里，因为实测东、北两向的
        # 有效速度不同（分段起步滑行让实际位移偏多）。
        east_ms = _ms(EAST_M, args.speed)
        north_ms = _ms(north_m, args.speed)
    else:
        # 默认用实标常数；改留缝距离时按北向实测速度折算。
        east_ms = EAST_MS
        north_ms = round(NORTH_MS * north_m / NORTH_M)
    if args.east_ms is not None:
        east_ms = args.east_ms
    if args.north_ms is not None:
        north_ms = args.north_ms

    gap_text = ("车头前沿贴住九宫格南边" if args.gap_cm == 0
                else f"车头前沿距九宫格南边 {args.gap_cm:.1f} cm")
    print(f"东行 {EAST_M * 100:.1f} cm（{east_ms} ms）→ 左转 {args.turn_deg:.1f}° → "
          f"北行 {north_m * 100:.1f} cm（{north_ms} ms）", flush=True)
    print(f"理论终点 ({BOARD_CENTER_M[0]:.2f}, {START_M[1] + north_m:.2f}) m，"
          f"车头朝 +Y，{gap_text}。", flush=True)
    print("把车放在左下角 (0.17, 0.17)、车头朝 +X，保持静止两秒。", flush=True)
    if args.dry_run:
        return

    try:
        with open_imu(args.imu_port, args.calibration_s, 0.) as imu, \
                ReconnectLink(args.port) as link:
            drive_to_board(imu=imu, link=link, east_ms=east_ms, north_ms=north_ms,
                           turn_deg=args.turn_deg, chunk_ms=args.chunk_ms,
                           start_skip_ms=args.start_skip_ms,
                           deadband_deg=args.deadband_deg,
                           ms_per_deg=args.ms_per_deg, yaw_scale=args.yaw_scale,
                           turn_deadband_deg=args.turn_deadband_deg)
    except MotionBlocked as error:
        raise SystemExit(f"底盘因避障保持停车：{error}\n")
    except ValueError as error:
        raise SystemExit(f"{error}\n")
    except KeyboardInterrupt:
        raise SystemExit(130, "路线已中断。\n")
    except (TimeoutError, OSError) as error:
        raise SystemExit(f"路线中止：{error}\n")


if __name__ == "__main__":
    main()
