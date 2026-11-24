"""读取实车四个 SR09 的状态。link 复用项目已有的 Uno 字节流。"""

import argparse
import json
from time import sleep

from . import DEFAULT_PORT, open_uno, wait_reply


DIRECTIONS = ("front", "back", "left", "right")


def scan(link):
    """只探测地址 ACK，返回本总线上有响应的候选七位地址。"""
    link.write(b"SONAR SCAN\n")
    fields = wait_reply(link, b"SCAN", fields=True).split()[1:]
    return [int(field, 16) for field in fields]


def read(link):
    """单次快照，各方向返回七位地址和距离（毫米），无有效数据为 None。"""
    link.write(b"SONAR\n")
    values = wait_reply(link, b"SONAR", fields=True).decode("ascii").split()[1:]
    if len(values) != len(DIRECTIONS):
        raise ValueError("Uno 测距回执缺少四向数据")
    result = {}
    for direction, item in zip(DIRECTIONS, values):
        address, distance = item.split("=", 1)
        result[direction] = {"address": address,
                             "mm": None if distance == "NA" else int(distance)}
    return result


def main():
    p = argparse.ArgumentParser(description="Uno 四向超声波调试，距离单位毫米")
    p.add_argument("action", choices=("scan", "read"))
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--watch", action="store_true", help="持续读取四向距离")
    args = p.parse_args()
    if args.watch and args.action != "read":
        p.error("--watch 适用于 read")
    try:
        with open_uno(args.port) as link:
            if args.action == "scan":
                print(" ".join(f"0x{a:02X}" for a in scan(link)) or "未发现测距地址")
            else:
                while True:
                    print(json.dumps(read(link)), flush=True)
                    if not args.watch:
                        break
                    sleep(0.25)
    except (OSError, ValueError) as error:
        p.exit(1, f"{error}\n")
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
