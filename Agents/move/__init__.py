"""银立方四轮底盘的树莓派串口接口。"""

from contextlib import nullcontext, suppress

if "." in __package__:
    from ..uno import (BAUD_RATE, DEFAULT_PORT, MotionBlocked, ReconnectLink, open_uno,
                       stop_wheels, wait_reply)
else:
    from uno import (BAUD_RATE, DEFAULT_PORT, MotionBlocked, ReconnectLink, open_uno,
                     stop_wheels, wait_reply)

DEFAULT_DURATION_MS = 1000
ACTIONS = (
    "go", "back", "left", "right",
    "forward-left", "forward-right", "back-left", "back-right",
    "rotate-left", "rotate-right", "turn-right-left-wheels", "turn-right-front-wheels",
    "m1", "m2", "m3", "m4", "stop",
)


def encode_command(action: str, duration_ms: int = DEFAULT_DURATION_MS) -> bytes:
    """将命令行或 Python 调用转换为 Uno 接收的一行 ASCII。"""
    action = action.lower()
    if action not in ACTIONS:
        raise ValueError(f"未知底盘动作 {action!r}")
    if action == "stop":
        return b"STOP\n"
    if type(duration_ms) is not int or not 1 <= duration_ms <= 65535:
        raise ValueError("运动时长必须是 1 至 65535 之间的整数毫秒数")
    return f"{action.upper()} {duration_ms}\n".encode("ascii")


def move(action: str, duration_ms: int = DEFAULT_DURATION_MS, *, port: str = DEFAULT_PORT,
         link=None, guarded=False) -> None:
    """执行定时动作并等待停车。link 可复用已与机械臂共用的 Uno 连接。"""
    command = encode_command(action, duration_ms)
    with (nullcontext(link) if link is not None else open_uno(port)) as connection:
        if command == b"STOP\n":
            if link is not None:
                stop_wheels(connection)
            return
        try:
            connection.write(b"NAV MOVE " + command if guarded else command)
            wait_reply(connection, b"OK MOVE")
            wait_reply(connection, b"DONE", duration_ms / 1000 + 1)
        except MotionBlocked:
            # 固件已切断四轮输出，保留阻挡原因交给上层决定下一步。
            raise
        except (KeyboardInterrupt, OSError):
            with suppress(OSError):
                stop_wheels(connection)
            raise


def avoidance(on: bool, *, port: str = DEFAULT_PORT, link=None) -> None:
    """切换固件避障开关。on=False 关闭避障，用于架空的点动或位移验收。"""
    command = b"AVOID ON\n" if on else b"AVOID OFF\n"
    expected = b"OK AVOID ON" if on else b"OK AVOID OFF"
    with (nullcontext(link) if link is not None else open_uno(port)) as connection:
        connection.write(command)
        wait_reply(connection, expected)
