"""总控流程：唤醒 → 听口令 → 南侧路线 → 闭环入格。

启动后只开语音识别，听到唤醒词之前不做任何相机检测，也不驱动车和臂。
唤醒后播报问候语，之后每识别出一句文字就立刻交给 DeepSeek 解析格号；
没解析出格号就继续听（不要求重新说唤醒词），解析成功后依次执行：

    播报“开始移动” → move.south_route → 播报“已抵达地点”
    → vision.place 闭环入格 → 播报“已抵达x号位” → 退出

避障在入口处关闭（AVOID OFF 只在本次通电有效），退出时恢复 AVOID ON。
实车运行前把车放在 (0.17, 0.17) m、车头朝 +X，九宫格在正东、南边对齐 y=0.75 m。

用法（在 src 目录、已激活 venv）：
    export DEEPSEEK_API_KEY=...
    python -m task                 # 正式运行
    python -m task --dry-run       # 只打印流程参数，不碰硬件
"""

import argparse
import json
import os
from contextlib import suppress
from pathlib import Path
from time import sleep

from listen_speak import (
    config as speech_config,
    is_online,
    recognize,
    say_event,
    understand,
    wait_for_wake,
)
from move import MotionBlocked, avoidance
from move.nav_sensors import open_imu
from move.south_route import EAST_MS, NORTH_MS, TURN_DEG, drive_to_board
from uno import DEFAULT_PORT, ReconnectLink


def _log(message):
    print(message, flush=True)


def parse_args():
    p = argparse.ArgumentParser(description="总控：语音唤醒 → 南侧路线 → 闭环入格")
    p.add_argument("--wake-timeout", type=float,
                   help=f"每轮等用户开口的秒数，默认 {speech_config.LISTEN_TIMEOUT}")
    p.add_argument("--window-s", type=float, default=speech_config.COMMAND_WINDOW_S,
                   help="唤醒后每轮最多等多久开口；识别出文字就立即交给大模型")
    p.add_argument("--echo-guard-s", type=float, default=.6,
                   help="播报问候后先等多久再开录，避免录到自己的声音")

    p.add_argument("--east-ms", type=int, default=EAST_MS, help="东行时长")
    p.add_argument("--north-ms", type=int, default=NORTH_MS, help="北行时长")
    p.add_argument("--turn-deg", type=float, default=TURN_DEG, help="原地左转角度")
    p.add_argument("--chunk-ms", type=int, default=400, help="航向保持的直行分段")
    p.add_argument("--ms-per-deg", type=float, default=45., help="短脉冲修正的毫秒每度")
    p.add_argument("--yaw-scale", type=float, default=1., help="陀螺航向比例修正")
    p.add_argument("--imu-port", default="/dev/ttyUSB0")
    p.add_argument("--calibration-s", type=float, default=2., help="IMU 静止标定时间")
    p.add_argument("--port", default=DEFAULT_PORT)

    p.add_argument("--record", help="南侧路线三阶段用前摄记录到墙标距离并写入该 JSONL 文件；缺省不记录")
    p.add_argument("--front-config", default="/home/mooc/front-cal-97Gvip/front-centered.json",
                   help="前摄内参 JSON（记录时用），默认刚标定好的 front-centered.json")
    p.add_argument("--front-source", default=os.environ.get("FRONT_CAM", "0"),
                   help="前摄设备，默认取环境变量 FRONT_CAM，否则 0")
    p.add_argument("--side-mm", type=float, default=200., help="墙标黑框边长，默认 200 mm")
    p.add_argument("--tag-before", type=int, default=3, help="左转前记录的前方墙标 id")
    p.add_argument("--tag-after", type=int, default=1, help="左转后记录的前方墙标 id")
    p.add_argument("--tag-end", type=int, default=1, help="北行结束记录的前方墙标 id")

    p.add_argument("--method", choices=("auto", "arm", "drive"), default="auto",
                   help="入格方式：auto 按可达性分流，arm/drive 强制只用一种")
    p.add_argument("--top-source", default=os.environ.get("TOP_CAM", "2"),
                   help="顶摄编号或设备端点，默认取环境变量 TOP_CAM，否则 2")
    p.add_argument("--config", help="顶摄安装参数 JSON，缺省按 1280×720")
    p.add_argument("--no-raw", dest="raw", action="store_false",
                   help="用 MJPG 压缩；默认为 YUYV（--raw），小角标更稳")
    p.add_argument("--targets", help="每格目标 JSON，转发给 vision.place")
    p.add_argument("--arm", type=int, nargs=2, metavar=("肩µs", "肘µs"),
                   help="车体闭环时臂的固定脉宽，默认 360 1570")
    p.add_argument("--release", action="store_true", help="结束后 ARM OFF，臂会失去保持力矩")
    p.add_argument("--dry-run", action="store_true", help="只打印流程参数，不碰硬件")
    return p.parse_args()


def describe(args):
    """dry-run：打印将要执行的流程，说明场地与依赖要求。"""
    print("总控流程（dry-run，不动作）：", flush=True)
    print(f"  唤醒词：{'、'.join(speech_config.WAKE_WORDS)}", flush=True)
    print(f"  口令窗口：{args.window_s:.1f} s，问候后等待 {args.echo_guard_s:.1f} s 再开录",
          flush=True)
    print(f"  南侧路线：东行 {args.east_ms} ms → 左转 {args.turn_deg:.1f}° → "
          f"北行 {args.north_ms} ms", flush=True)
    print(f"  IMU：{args.imu_port}，静止标定 {args.calibration_s:.1f} s", flush=True)
    print(f"  入格：method={args.method}，顶摄 source={args.top_source}", flush=True)
    if args.record:
        print(f"  前摄记录：写入 {args.record}（config={args.front_config}，source={args.front_source}）",
              flush=True)
        print(f"    记录墙标：左转前 id={args.tag_before}，左转后 id={args.tag_after}，"
              f"北行结束 id={args.tag_end}", flush=True)
    else:
        print("  前摄记录：关闭（给 --record 指定文件后启用）", flush=True)
    print(f"  串口：{args.port}", flush=True)
    key = "已设置" if speech_config.LLM_API_KEY else "未设置（正式运行前 export DEEPSEEK_API_KEY）"
    print(f"  DeepSeek Key：{key}", flush=True)


def wait_for_command(args):
    """等唤醒词，之后逐句识别并解析，返回 0–8 的格号。

    唤醒一次后持续监听：只要识别出文字就立即交给大模型，不等 7 s 窗口攒句子。
    没解析出格号就在同一次唤醒里继续听，不要求再说“小伙小伙”；
    只有噪声、没识别出文字的轮次直接跳过，不播报、不打断。
    """
    while True:
        _log("等待唤醒词……")
        wait_for_wake(args.wake_timeout)
        _log("收到唤醒词。")
        say_event("wake")
        sleep(args.echo_guard_s)  # 等播报播完，避免把自己的声音录进口令
        while True:
            text = recognize(args.window_s)
            if not text:
                continue
            _log(f"交给大模型解析：{text}")
            cell, _action = understand(text)
            _log(f"解析结果：{cell if cell is not None else '（无格号）'}")
            if cell is not None:
                return int(cell)
            say_event("offline" if not is_online() else "no_command")


class PhaseRecorder:
    """南侧路线三阶段用前摄测距并记录到 JSONL；只读相机、不发运动，失败不中断主流程。"""

    def __init__(self, config_path, source, side_mm, record_path, tags):
        from move.tag_route import TagRanger
        from vision.camera import Mount

        mount = Mount.load(config_path)
        self.ranger = TagRanger(source, mount, tag_id=None, side_mm=side_mm, family="36h11")
        self.out = open(record_path, "a", encoding="utf-8")
        self.tags = tags

    def __call__(self, moment):
        try:
            tag_id = self.tags[moment]
            self.ranger.tag_id = tag_id
            fix = self.ranger.measure()
            if fix is None:
                entry = {"moment": moment, "tag_id": tag_id, "detected": False}
            else:
                entry = {"moment": moment, "tag_id": fix.tag_id, "z_mm": round(fix.z_mm, 1),
                         "x_mm": round(fix.x_mm, 1), "y_mm": round(fix.y_mm, 1),
                         "rms_px": round(fix.rms_px, 2)}
            line = json.dumps(entry, ensure_ascii=False)
            _log(f"[记录][{moment}] {line}")
            self.out.write(line + "\n")
            self.out.flush()
        except Exception as error:
            # 记录是旁路功能：任何失败都不能影响后续运动。
            _log(f"[记录][{moment}] 失败（不影响运动）：{error}")

    def close(self):
        self.ranger.close()
        self.out.close()


def place_cell(args, link, cell):
    """在已停稳的南侧原位做一次闭环入格，返回该格的 CellResult。"""
    from vision.camera import Mount
    from vision.drive import DRIVE_PULSES, DriveConfig
    from vision.place import place_cells
    from vision.servo import ServoConfig
    from vision.tags import Detector

    mount = Mount.load(args.config)
    if args.raw:
        mount.raw = True
    if not mount.image_size:
        mount.image_size = (1280, 720)
    targets = (json.loads(Path(args.targets).read_text(encoding="utf-8"))
               if args.targets else None)
    pulses = tuple(args.arm) if args.arm else DRIVE_PULSES
    results = place_cells([cell], method=args.method, source=args.top_source, mount=mount,
                          link=link, detector=Detector(("36h11", "36h10")),
                          servo_config=ServoConfig(), drive_config=DriveConfig(),
                          targets=targets, pulses=pulses, release=args.release,
                          log=_log)
    return results[0]


def run(args, link):
    """一次完整任务：听口令 → 路线 → 入格 → 播报。返回进程退出码。"""
    cell = wait_for_command(args)
    say_event("moving")
    recorder = None
    if args.record:
        try:
            recorder = PhaseRecorder(args.front_config, args.front_source, args.side_mm,
                                     args.record, {"before_turn": args.tag_before,
                                                   "after_turn": args.tag_after,
                                                   "end_north": args.tag_end})
            _log(f"前摄记录已启用，写入 {args.record}。")
        except (OSError, ValueError) as error:
            _log(f"前摄记录不可用，跳过记录（不影响运动）：{error}")
            recorder = None
    phase_event = {"before_turn": "turn", "end_north": "dock"}

    def on_phase(moment):
        """阶段回调：若启用了记录则顺带测距；并在两处 AprilTag 地点播报固定提示。"""
        if recorder is not None:
            recorder(moment)
        event = phase_event.get(moment)
        if event:
            say_event(event)

    try:
        with open_imu(args.imu_port, args.calibration_s, 0.) as imu:
            drive_to_board(imu=imu, link=link, east_ms=args.east_ms, north_ms=args.north_ms,
                           turn_deg=args.turn_deg, chunk_ms=args.chunk_ms,
                           ms_per_deg=args.ms_per_deg, yaw_scale=args.yaw_scale,
                           log=_log, on_phase=on_phase)
    finally:
        if recorder is not None:
            with suppress(OSError):
                recorder.close()
    say_event("arrived")

    result = place_cell(args, link, cell)
    if not result.ok:
        _log(f"格 {cell}（{result.method}）未完成：{result.note}")
        say_event("end")
        return 1
    _log(f"格 {cell} 到位，残余 {result.error_mm:.1f} mm，方式 {result.method}。")
    say_event("cell", number=cell)
    return 0


def main():
    args = parse_args()
    if args.dry_run:
        describe(args)
        return 0
    if not speech_config.LLM_API_KEY:
        print("未设置 DEEPSEEK_API_KEY，无法解析口令。先 export DEEPSEEK_API_KEY=...",
              flush=True)
        return 2

    try:
        with ReconnectLink(args.port) as link:
            avoidance(False, link=link)
            try:
                return run(args, link)
            finally:
                # 链路已断时清理动作也会失败，这里只尽力恢复避障并让原异常继续上抛。
                with suppress(OSError, TimeoutError):
                    avoidance(True, link=link)
    except KeyboardInterrupt:
        _log("任务已中断。")
        return 130
    except (MotionBlocked, TimeoutError, OSError, ValueError, RuntimeError) as error:
        _log(f"任务中止：{error}")
        say_event("end")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
