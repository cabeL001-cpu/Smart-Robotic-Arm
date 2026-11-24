"""IMU 航向保持直行：分段前进，停车后按累计航向偏差原地修正。

Uno 的直行动作只有定速档，轮速差会让车头逐渐偏转（实车表现为左偏）。
这里把一次长直行切成短段，每段停车后用陀螺仪积分的累计转角测出航向偏差，
再按标定的毫秒每度点动 `rotate-left`/`rotate-right` 转回。

不用 AHRS 四元数的航向角：实车磁力计被电机、电池和车架干扰，手推 90°
四个方向分别只读 59°、75°、142°、83°，停车静止时读数还会自己回落几度。
陀螺积分只看转动本身，不受这些静态磁场影响。
"""

import argparse
import math
from time import sleep

from . import DEFAULT_PORT, MotionBlocked, ReconnectLink, move
from .imu import wrap
from .nav_sensors import open_imu


MS_PER_DEG = 4000. / 90.    # 实测 90° 约 4 s，用于估算修正脉冲时长
SETTLE_S = .5               # Uno 回 DONE 后等滑行和回弹结束再读陀螺


def _send(action, pulse_ms, link):
    """发一次点动。蓝牙偶发丢失回执时 move() 已尝试停车，返回 False 让上层重测。"""
    try:
        move(action, pulse_ms, link=link)
    except TimeoutError:
        print(f"  {action} {pulse_ms} ms 未收到回执，已尝试停车", flush=True)
        return False
    return True


def _heading_deg(turn, yaw_scale):
    """把 IMU 累计转角换算成实车角度。yaw_scale = 90 / 手推 90° 的实测读数。"""
    return math.degrees(wrap(turn)) * yaw_scale


def _error_deg(target_deg, turn, yaw_scale):
    """实车航向偏差，范围 ±180°，正表示需要左转。"""
    error = target_deg - _heading_deg(turn, yaw_scale)
    return math.degrees(wrap(math.radians(error)))


def read_heading(imu, yaw_scale=1.):
    """相对 IMU 静止标定时刻的累计航向，单位度，左转为正。"""
    _, turn = imu.snapshot_gyro()
    return _heading_deg(turn, yaw_scale)


def correct_heading(imu, link, *, target_deg=0., deadband_deg=2., ms_per_deg=MS_PER_DEG,
                    yaw_scale=1., min_pulse_ms=65, max_pulse_ms=1000,
                    settle_s=SETTLE_S, attempts=4):
    """测量当前航向，超过死区就点动转回。返回停车后的最终偏差（度）。"""
    for _ in range(attempts):
        sleep(settle_s)
        _, turn = imu.snapshot_gyro()
        error = _error_deg(target_deg, turn, yaw_scale)
        if abs(error) <= deadband_deg:
            return error
        # 左偏使累计航向为正、误差为负，此时右转；右偏反之。
        pulse_ms = round(min(max_pulse_ms, max(min_pulse_ms, abs(error) * ms_per_deg)))
        side = "右" if error < 0 else "左"
        print(f"  航向偏差 {error:+.1f}°，{side}转 {pulse_ms} ms", flush=True)
        if not _send("rotate-right" if error < 0 else "rotate-left", pulse_ms, link):
            continue
    sleep(settle_s)
    _, turn = imu.snapshot_gyro()
    return _error_deg(target_deg, turn, yaw_scale)


def turn_to(target_deg, *, imu, link, deadband_deg=1., coarse_ms_per_deg=MS_PER_DEG,
            fine_ms_per_deg=70., yaw_scale=1., min_pulse_ms=65, max_pulse_ms=4000,
            settle_s=SETTLE_S, attempts=8):
    """原地转到相对启动时刻的目标航向（左转为正），返回最终偏差。

    第一次按 90°≈4000 ms 的长脉冲粗转；残留的小角度改用实车短脉冲增益，
    短脉冲起步有加速过程，按 44.4 ms/度会明显转不到位。
    """
    for attempt in range(attempts):
        sleep(settle_s)
        _, turn = imu.snapshot_gyro()
        error = _error_deg(target_deg, turn, yaw_scale)
        if abs(error) <= deadband_deg:
            return error
        gain = coarse_ms_per_deg if attempt == 0 else fine_ms_per_deg
        pulse_ms = round(min(max_pulse_ms, max(min_pulse_ms, abs(error) * gain)))
        side = "左" if error > 0 else "右"
        print(f"  转向偏差 {error:+.1f}°，{side}转 {pulse_ms} ms", flush=True)
        if not _send("rotate-left" if error > 0 else "rotate-right", pulse_ms, link):
            continue
    sleep(settle_s)
    _, turn = imu.snapshot_gyro()
    return _error_deg(target_deg, turn, yaw_scale)


def go_straight(duration_ms, *, imu, link, action="go", chunk_ms=400,
                target_deg=0., skip_correction_ms=0, **correction):
    """按 chunk_ms 分段前进或后退，每段之间用 IMU 修正航向，返回最终偏差。

    skip_correction_ms 用于起点后方不足 30 cm 的场合：先连续直行离开边界，
    等旋转不再被避障拦截后再开始修正。
    """
    if action not in ("go", "back"):
        raise ValueError("航向保持只用于 go 或 back")
    if not 1 <= chunk_ms <= duration_ms:
        raise ValueError("分段时长必须为正且不超过总时长")
    remaining = duration_ms
    forward = 0
    segment = 0
    error = 0.
    while remaining > 0:
        segment += 1
        ms = min(chunk_ms, remaining)
        _send(action, ms, link)
        remaining -= ms
        forward += ms
        if forward < skip_correction_ms:
            print(f"第 {segment} 段 {ms} ms，剩余 {remaining} ms，跳过修正", flush=True)
            continue
        error = correct_heading(imu, link, target_deg=target_deg, **correction)
        print(f"第 {segment} 段 {ms} ms，剩余 {remaining} ms，"
              f"航向偏差 {error:+.1f}°", flush=True)
    return error


def watch_heading(imu, yaw_scale=1.):
    """不接 Uno，同时打印陀螺航向和 AHRS 航向，用于核对方向、符号和比例。

    两列差值持续变大或手停后 AHRS 自己走，说明磁力计在拉偏，只能信陀螺列。
    """
    while True:
        _, gyro_turn = imu.snapshot_gyro()
        _, ahrs_turn = imu.snapshot()
        print(f"陀螺 {_heading_deg(gyro_turn, yaw_scale):+.1f}°  "
              f"AHRS {_heading_deg(ahrs_turn, 1.):+.1f}°", flush=True)
        sleep(.25)


def main():
    parser = argparse.ArgumentParser(description="IMU 航向保持的分段直行")
    parser.add_argument("--ms", type=int, default=6000, help="前进总时长，不含修正点动")
    parser.add_argument("--action", choices=("go", "back"), default="go")
    parser.add_argument("--target-deg", type=float, default=0.,
                        help="相对启动时的目标航向，左转为正")
    parser.add_argument("--turn-deg", type=float,
                        help="只原地转到该角度，不直行，用于单独验收转向")
    parser.add_argument("--chunk-ms", type=int, default=400, help="每段直行时长")
    parser.add_argument("--skip-correction-ms", type=int, default=0,
                        help="起步阶段先不修正的毫秒数，用于后方净空不足的起点")
    parser.add_argument("--deadband-deg", type=float, default=2., help="允许的航向偏差")
    parser.add_argument("--ms-per-deg", type=float, default=MS_PER_DEG,
                        help="原地转动的毫秒每度，默认按 90°≈4000 ms")
    parser.add_argument("--yaw-scale", type=float, default=1.,
                        help="陀螺航向比例修正，等于 90 除以手推 90° 时陀螺列读数")
    parser.add_argument("--imu-port", default="/dev/ttyUSB0")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--calibration-s", type=float, default=2., help="IMU 静止标定时间")
    parser.add_argument("--watch", action="store_true", help="只打印航向，不驱动底盘")
    args = parser.parse_args()

    try:
        if args.watch:
            with open_imu(args.imu_port, args.calibration_s, 0.) as imu:
                print("IMU 已标定。", flush=True)
                watch_heading(imu, args.yaw_scale)
        elif args.turn_deg is not None:
            with open_imu(args.imu_port, args.calibration_s, 0.) as imu, \
                    ReconnectLink(args.port) as link:
                print(f"IMU 已标定，原地转向 {args.turn_deg:+.1f}°。", flush=True)
                error = turn_to(args.turn_deg, imu=imu, link=link,
                                deadband_deg=args.deadband_deg,
                                fine_ms_per_deg=args.ms_per_deg,
                                yaw_scale=args.yaw_scale)
                print(f"转向结束，最终偏差 {error:+.1f}°。", flush=True)
        else:
            with open_imu(args.imu_port, args.calibration_s, 0.) as imu, \
                    ReconnectLink(args.port) as link:
                print(f"IMU 已标定，目标航向 {args.target_deg:+.1f}°。", flush=True)
                error = go_straight(args.ms, imu=imu, link=link, action=args.action,
                                    chunk_ms=args.chunk_ms, target_deg=args.target_deg,
                                    skip_correction_ms=args.skip_correction_ms,
                                    deadband_deg=args.deadband_deg,
                                    ms_per_deg=args.ms_per_deg,
                                    yaw_scale=args.yaw_scale)
                print(f"直行结束，最终航向偏差 {error:+.1f}°。", flush=True)
    except MotionBlocked as error:
        raise SystemExit(f"底盘因避障保持停车：{error}\n")
    except ValueError as error:
        raise SystemExit(f"{error}\n")
    except KeyboardInterrupt:
        raise SystemExit(130, "动作已中断。\n")
    except (TimeoutError, OSError) as error:
        raise SystemExit(f"航向保持中止：{error}\n")


if __name__ == "__main__":
    main()
