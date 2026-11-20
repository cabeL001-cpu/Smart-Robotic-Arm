"""底盘和机械臂共用的 Uno 串口。一次打开可依次执行多个模块的命令。"""

from contextlib import contextmanager
from time import monotonic

DEFAULT_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600


class MotionBlocked(OSError):
    """Uno 已因方向净空不足或测距缺失而保持停车。"""


def stop_wheels(link):
    # 首次打开 USB 可能触发 Uno 复位。停车握手也兼容原底盘固件。
    deadline = monotonic() + 4
    while monotonic() < deadline:
        link.write(b"STOP\n")
        if link.readline().strip() == b"OK STOP":
            return
    raise TimeoutError("Uno 未确认停车，请检查串口名称、波特率和固件")


def wait_reply(link, expected: bytes, seconds: float = 1, *, fields=False):
    deadline = monotonic() + seconds
    while monotonic() < deadline:
        reply = link.readline().strip()
        if reply == expected or (fields and reply.startswith(expected + b" ")):
            return reply if fields else None
        if reply.startswith((b"ERR OBSTACLE ", b"ERR SONAR ")):
            raise MotionBlocked(reply.decode("ascii", errors="replace"))
        if reply and reply != b"OK STOP":
            raise OSError(f"Uno 返回 {reply!r}，期望 {expected!r}")
    raise TimeoutError(f"等待 Uno 回执 {expected!r} 超时")


@contextmanager
def open_uno(port=DEFAULT_PORT):
    """建立独占串口并停车同步。调用方在同一连接上顺序执行动作。"""
    import serial

    with serial.Serial(port, BAUD_RATE, timeout=0.25, write_timeout=1, exclusive=True) as link:
        link.reset_input_buffer()
        stop_wheels(link)
        yield link
