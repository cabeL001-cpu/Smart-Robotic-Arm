"""九宫格入格总入口：格在臂可达集内用臂闭环，够不到时改用底盘微调。

可达集取 servo.CELL_SEEDS 的键（0、3、6、7、8）；1、2、4、5 交给车体闭环。
method 可显式指定 arm 或 drive，跳过自动分流。一次串口、一路相机逐格执行，
每格返回统一的 CellResult，带 method 标记实际使用的方式。
"""

import argparse
from contextlib import nullcontext
from dataclasses import asdict, dataclass
import json
import math

import numpy as np

from .camera import Mount
from .drive import DRIVE_PULSES, DriveConfig, Driver
from .servo import CELL_SEEDS, Servo, ServoConfig
from .tags import Detector

if "." in __package__:
    from ..uno import DEFAULT_PORT, open_uno
else:
    from uno import DEFAULT_PORT, open_uno

# 臂单独能够到的格：11-23 实测 0、3、6、7、8；其余格需要走车调整。
ARM_CELLS = frozenset(CELL_SEEDS)
METHODS = ("auto", "arm", "drive")


@dataclass(frozen=True)
class CellResult:
    cell: int
    ok: bool
    method: str          # "arm" 或 "drive"
    error_mm: float
    pulses: tuple        # 该格结束时的臂脉宽
    moves: int           # 下发动作次数
    rms_px: float
    note: str = ""


def place_cells(cells, *, method="auto", source, mount=None, port=DEFAULT_PORT, link=None,
                detector=None, servo_config=ServoConfig(), drive_config=DriveConfig(),
                targets=None, pulses=DRIVE_PULSES, release=False, log=print):
    """按顺序送完各格。auto：可达格走臂，其余走车体；arm/drive 强制只用一种。"""
    if method not in METHODS:
        raise ValueError(f"未知入格方式 {method!r}，可选 {METHODS}")
    with (nullcontext(link) if link is not None else open_uno(port)) as connection:
        servo = Servo(connection, source, mount, detector=detector, config=servo_config, log=log)
        try:
            if targets:
                servo.targets.update({int(cell): np.asarray(xy, dtype=float)
                                      for cell, xy in targets.items()})
            missing = [cell for cell in cells if cell not in servo.targets]
            if missing:
                servo.measure_targets(range(9))
            driver = Driver(servo, config=drive_config, pulses=pulses, log=log)
            results, held = [], None
            for cell in cells:
                if method == "arm" or (method == "auto" and cell in ARM_CELLS):
                    placement = servo.place(cell, CELL_SEEDS.get(cell) or held)
                    held = (placement.shoulder_us, placement.elbow_us)
                    result = CellResult(cell, placement.ok, "arm", placement.error_mm,
                                        held, placement.moves, placement.rms_px, placement.note)
                else:
                    driven = driver.place(cell)
                    held = driven.pulses
                    result = CellResult(cell, driven.ok, "drive", driven.error_mm,
                                        driven.pulses, driven.moves, driven.rms_px, driven.note)
                results.append(result)
                if not result.ok:
                    residual = "" if math.isnan(result.error_mm) else \
                        f"，残余 {result.error_mm:.1f} mm"
                    log(f"格 {cell}（{result.method}）未完成：{result.note}{residual}")
            if release:
                servo.release()
            return results
        finally:
            servo.close()


def main():
    p = argparse.ArgumentParser(description="九宫格入格总入口：可达格用臂，其余用车体微调")
    p.add_argument("--cells", type=int, nargs="+", default=[4], choices=range(9))
    p.add_argument("--method", choices=METHODS, default="auto",
                   help="auto 按可达性分流；arm/drive 强制只用一种")
    p.add_argument("--source", default="0")
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--config", help="安装参数 JSON，缺省按 1280×720 顶摄")
    p.add_argument("--probe", type=int, default=60, help="臂雅可比探测脉宽")
    p.add_argument("--probe-sign", type=int, nargs=2, choices=(-1, 1), metavar=("肩", "肘"),
                   help="臂探测方向，取 -1 往脉宽减小方向探")
    p.add_argument("--tolerance", type=float, default=4.0, help="臂到位误差 mm")
    p.add_argument("--max-steps", type=int, default=6, help="臂每格迭代上限")
    p.add_argument("--shoulder", type=int, nargs=2, metavar=("最小µs", "最大µs"),
                   help="肩部脉宽窗口，默认 360–640")
    p.add_argument("--elbow", type=int, nargs=2, metavar=("最小µs", "最大µs"),
                   help="肘部脉宽窗口，默认 500–2500")
    p.add_argument("--arm", type=int, nargs=2, default=list(DRIVE_PULSES), metavar=("肩µs", "肘µs"),
                   help="车体闭环时臂的固定脉宽，默认 360 1570")
    p.add_argument("--drive-probe", type=int, default=300, help="测车体雅可比的底盘动作时长 ms")
    p.add_argument("--drive-sign", type=int, nargs=2, choices=(-1, 1), metavar=("前进", "左移"),
                   help="车体探测方向，-1 改为后退/右移")
    p.add_argument("--drive-step", type=int, default=700, help="单次走车修正时长上限 ms")
    p.add_argument("--drive-tolerance", type=float, default=5.0, help="车体到位误差 mm")
    p.add_argument("--drive-steps", type=int, default=8, help="车体每格迭代上限")
    p.add_argument("--targets", help="每格目标 JSON，如 {\"4\": [100.3, 100.0]}")
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
    servo_config = ServoConfig(probe_us=args.probe, tolerance_mm=args.tolerance,
                               max_steps=args.max_steps,
                               probe_sign=tuple(args.probe_sign or (1, 1)), **windows)
    drive_config = DriveConfig(probe_ms=args.drive_probe,
                               probe_sign=tuple(args.drive_sign or (1, 1)),
                               max_step_ms=args.drive_step, tolerance_mm=args.drive_tolerance,
                               max_steps=args.drive_steps)
    targets = json.load(open(args.targets, encoding="utf-8")) if args.targets else None

    results = place_cells(args.cells, method=args.method, source=args.source, mount=mount,
                          port=args.port, detector=Detector(("36h11", "36h10")),
                          servo_config=servo_config, drive_config=drive_config,
                          targets=targets, pulses=tuple(args.arm), release=args.release)
    for result in results:
        print(json.dumps(asdict(result), ensure_ascii=False))


if __name__ == "__main__":
    main()