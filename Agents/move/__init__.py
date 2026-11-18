"""银立方四轮底盘的树莓派串口接口。"""

from contextlib import suppress
from time import monotonic

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_DURATION_MS = 1000
BAUD_RATE = 9600
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


def _sync_stop(link):
    # 打开 USB 串口可能使 Uno 复位。重复发送可随时执行的 STOP，直到固件应答。
    deadline = monotonic() + 4
    while monotonic() < deadline:
        link.write(b"STOP\n")
        if link.readline().strip() == b"OK STOP":
            return
    raise TimeoutError("Uno 未确认停车，请检查串口名称、波特率和底盘固件")


def _wait_for(link, expected: bytes, seconds: float):
    deadline = monotonic() + seconds
    while monotonic() < deadline:
        reply = link.readline().strip()
        if reply == expected:
            return
        # 启动握手可能留下多条停车回执，按串口发送顺序读完即可。
        if reply and reply != b"OK STOP":
            raise OSError(f"Uno 返回 {reply!r}，期望 {expected!r}")
    raise TimeoutError(f"等待 Uno 回执 {expected!r} 超时")


def move(action: str, duration_ms: int = DEFAULT_DURATION_MS, *, port: str = DEFAULT_PORT) -> None:
    """按给定时长执行一个动作，收到 Uno 的停车回执后返回。stop 可立即停车。"""
    command = encode_command(action, duration_ms)
    import serial  # 仅操作串口时需要 pyserial，预览指令可直接使用系统 Python。

    with serial.Serial(port, BAUD_RATE, timeout=0.25, write_timeout=1, exclusive=True) as link:
        link.reset_input_buffer()
        _sync_stop(link)
        if command == b"STOP\n":
            return
        try:
            link.write(command)
            _wait_for(link, b"OK MOVE", 1)
            _wait_for(link, b"DONE", duration_ms / 1000 + 1)
        except (KeyboardInterrupt, OSError):
            # 中断或回执异常时请求停车。串口已断开时，Uno 自身的计时仍会到期。
            with suppress(OSError):
                _sync_stop(link)
            raise
