"""在 Agents 目录运行 python3 -m move。"""

import argparse

from . import ACTIONS, DEFAULT_DURATION_MS, DEFAULT_PORT, encode_command, move


def main():
    parser = argparse.ArgumentParser(description="通过 Uno 串口驱动银立方四轮底盘")
    parser.add_argument("action", type=str.lower, choices=ACTIONS, help="底盘动作")
    parser.add_argument("--ms", type=int, default=DEFAULT_DURATION_MS,
                        help="运动时长，默认 1000 毫秒。stop 立即执行")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"串口设备，默认 {DEFAULT_PORT}")
    parser.add_argument("--dry-run", action="store_true", help="在本机打印指令")
    args = parser.parse_args()

    try:
        if args.dry_run:
            print(encode_command(args.action, args.ms).decode("ascii"), end="")
        else:
            move(args.action, args.ms, port=args.port)
            print("Uno 已确认停车。")
    except ValueError as error:
        parser.error(str(error))
    except ModuleNotFoundError as error:
        parser.exit(1, f"{error}。请在当前 Python 环境安装 move/requirements.txt 中的依赖。\n")
    except OSError as error:
        parser.exit(1, f"串口控制失败：{error}\n")
    except KeyboardInterrupt:
        parser.exit(130, "动作已中断。\n")


if __name__ == "__main__":
    main()
