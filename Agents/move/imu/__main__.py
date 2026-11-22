"""实时 IMU 定位、原始样本记录和离线回放。"""

import argparse
from contextlib import ExitStack
from dataclasses import asdict
import json
import math
from queue import SimpleQueue, Empty
import sys
from threading import Thread

from . import Tracker
from .sensor import read_csv, record_samples, serial_samples


def fixes(lines):
    for line in lines:
        msg = json.loads(line)
        if msg.get("pose") is not None:
            pose = msg["pose"]
            yield msg["t"], pose["x_m"], pose["y_m"], pose["yaw_rad"]


def forward_input(lines, queue):
    try:
        for fix in fixes(lines):
            queue.put(fix)
    except (ValueError, KeyError) as error:
        queue.put(error)


def main():
    p = argparse.ArgumentParser(description="亚博 YbImu 九轴串口定位与回放")
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="IMU 串口，例如 /dev/ttyUSB0")
    source.add_argument("--replay", help="原始样本 CSV")
    p.add_argument("--record", help="另存原始样本 CSV")
    p.add_argument("--origin", type=float, nargs=3, default=(0.,0.,0.), metavar=("X米","Y米","车头角度"))
    p.add_argument("--mount-yaw", type=float, default=0., help="IMU +X 相对车头的逆时针角度")
    p.add_argument("--calibration-seconds", type=float, default=2.)
    p.add_argument("--stationary", action="store_true", help="台架静止测试，持续给出零速约束")
    aid = p.add_mutually_exclusive_group()
    aid.add_argument("--vision-stdin", action="store_true", help="实时读取 vision.robot 或 vision.front 的位姿 JSON 行")
    aid.add_argument("--fixes", help="回放同一时基记录的视觉 JSON 行")
    args = p.parse_args()
    if args.calibration_seconds <= 0:
        p.error("校准时间应为正数")
    if args.vision_stdin and args.replay:
        p.error("回放时请用 --fixes 对齐已记录的视觉时间戳")

    with ExitStack() as stack:
        if args.replay:
            samples = read_csv(stack.enter_context(open(args.replay, encoding="utf-8-sig", newline="")))
        else:
            import serial
            link = stack.enter_context(serial.Serial(args.port, 115200, timeout=.1, exclusive=True))
            link.reset_input_buffer()
            samples = serial_samples(link)
        if args.record:
            samples = record_samples(samples, stack.enter_context(open(args.record, "x", encoding="utf-8", newline="")))
        live_fixes = SimpleQueue()
        if args.vision_stdin:
            Thread(target=forward_input, args=(sys.stdin, live_fixes), daemon=True).start()
        recorded = fixes(stack.enter_context(open(args.fixes, encoding="utf-8-sig"))) if args.fixes else iter(())
        pending = next(recorded, None)
        calibration = []
        for s in samples:
            calibration.append(s)
            if s.t-calibration[0].t >= args.calibration_seconds:
                break
        if len(calibration) < 2 or calibration[-1].t-calibration[0].t < args.calibration_seconds:
            p.exit(1, "校准时间段内的原始样本不足。\n")
        x, y, yaw = args.origin
        tracker = Tracker(calibration, x_m=x, y_m=y, yaw_rad=math.radians(yaw),
                          mount_yaw_rad=math.radians(args.mount_yaw))
        for s in samples:
            tracker.update(s, stationary=args.stationary)
            corrected = False
            while True:
                if pending is None:
                    try:
                        pending = live_fixes.get_nowait()
                    except Empty:
                        break
                if isinstance(pending, Exception):
                    raise pending
                if pending[0] > s.t:
                    break
                corrected = tracker.correct(*pending) or corrected
                pending = next(recorded, None)
            print(json.dumps({**asdict(tracker.state), "vision_fixed": corrected},
                             allow_nan=False), flush=True)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError) as error:
        sys.exit(str(error))
