"""底盘和机械臂共用的 Uno 串口。一次打开可依次执行多个模块的命令。

默认经 HC-05 蓝牙 SPP 连接（/dev/rfcomm0）。台架调试时可用环境变量
UNO_PORT 或命令行的 --port 改回 USB 串口（如 /dev/ttyACM0）。
"""

import os
import shutil
import subprocess
import sys
from contextlib import contextmanager, suppress
from time import monotonic, sleep

DEFAULT_PORT = os.environ.get("UNO_PORT", "/dev/rfcomm0")
# 须与 HC-05 的 UART 波特率一致。模块出厂为 9600，改过则用 AT+UART=9600,0,0 改回。
BAUD_RATE = 9600
# 打开 /dev/rfcomm0 只是登记节点，RFCOMM 链路真正连通还要 2–4 秒，期间写入会丢失。
# 握手窗口要覆盖这段建链时间，实车实测约 3.7 秒，留足余量。
HANDSHAKE_SECONDS = 15


class MotionBlocked(OSError):
    """Uno 已因方向净空不足或测距缺失而保持停车。"""


_power_checked = False


def warn_power():
    """5V 欠压会让 HC-05 或 Uno 在底盘电机启动时掉线，每个进程只报一次。"""
    global _power_checked
    if _power_checked:
        return
    _power_checked = True
    if not shutil.which("vcgencmd"):
        return
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                             text=True, timeout=2).stdout.strip()
        value = int(out.split("=", 1)[1], 16)
    except (OSError, ValueError, IndexError, subprocess.SubprocessError):
        return
    if value & (0x1 | 0x10000):
        print(f"警告：树莓派 5V 欠压/降频（{out}），蓝牙和 Uno 会随机掉线；"
              "先换 5V/3A 电源或给 HC-05 单独供电", file=sys.stderr)


def stop_wheels(link, deadline=None):
    # 蓝牙链路刚建立时开头几个字节可能丢失，持续重发直到 Uno 确认停车。
    deadline = deadline or monotonic() + HANDSHAKE_SECONDS
    while monotonic() < deadline:
        link.write(b"STOP\n")
        if link.readline().strip() == b"OK STOP":
            return
    raise TimeoutError("Uno 未确认停车，请检查蓝牙绑定、波特率和固件")


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
    raise TimeoutError(
        f"等待 Uno 回执 {expected!r} 超时；检查 HC-05 链路（rfcomm -a、hcitool con）"
        "和树莓派 5V 供电（vcgencmd get_throttled，0x50005 表示欠压）")


def _check_port(port):
    """蓝牙串口要先绑定才会出现设备节点，缺失时提前给出可操作的提示。"""
    if port.startswith("/dev/") and not os.path.exists(port):
        raise OSError(
            f"{port} 不存在。HC-05 需先在树莓派上配对并绑定："
            "在 Agents 目录运行 sh bt_link.sh bind，或手工执行 "
            "sudo rfcomm bind rfcomm0 <MAC> 1。USB 台架调试可设 UNO_PORT=/dev/ttyACM0。"
        )


def _open_port(serial, port):
    try:
        return serial.Serial(port, BAUD_RATE, timeout=0.25, write_timeout=1, exclusive=True)
    except serial.SerialException as error:
        hint = ""
        if port.startswith("/dev/rfcomm"):
            hint = ("；权限不足时执行 sudo chown $USER " + port
                    + "；提示 exclusively lock 表示端口被其他进程占用，用 fuser -v " + port + " 查")
        raise OSError(f"{port} 打不开：{error}{hint}") from error


@contextmanager
def open_uno(port=DEFAULT_PORT):
    """建立独占串口并停车同步。调用方在同一连接上顺序执行动作。"""
    import serial

    _check_port(port)
    warn_power()
    deadline = monotonic() + HANDSHAKE_SECONDS
    while True:
        link = _open_port(serial, port)
        try:
            link.reset_input_buffer()
            stop_wheels(link, deadline)
        except (serial.SerialException, TimeoutError):
            # 上一个进程刚退出时模块仍在重连，节点会短暂报“设备已断开”。
            # 关掉节点重新打开，让内核重新发起 RFCOMM 连接。
            link.close()
            if monotonic() >= deadline:
                raise
            sleep(1)
        else:
            break
    with link:
        yield link


class ReconnectLink:
    """长路线用：蓝牙链路瞬时断开时重开端口并重新停车握手。

    只有写入或读取报“设备已断开”才触发。重握手的 STOP 会终止正在执行的动作，
    本次写入随后重发；若原指令其实已经执行，修正点动最多重复一次小脉冲，
    直行分段最多多走一段。
    """

    def __init__(self, port=DEFAULT_PORT):
        self.port = port
        self.stack = None
        self.link = None
        self.reopen()

    def reopen(self):
        self.close()
        deadline = monotonic() + HANDSHAKE_SECONDS
        while True:
            try:
                stack = open_uno(self.port)
                self.link = stack.__enter__()
            except OSError:
                # 蓝牙通道刚断开时内核可能还占着节点，排他锁约一秒后才释放，
                # 稍等重开；超过握手窗口才把错误交给上层。
                if monotonic() >= deadline:
                    raise
                sleep(.5)
                continue
            self.stack = stack
            return

    def close(self):
        link, self.link = self.link, None
        if link is not None:
            # 先直接关描述符，让内核立刻释放排他锁，再退出 open_uno 生成器。
            with suppress(Exception):
                link.close()
        stack, self.stack = self.stack, None
        if stack is not None:
            with suppress(Exception):
                stack.__exit__(None, None, None)

    def write(self, data):
        return self._retry(lambda: self.link.write(data))

    def readline(self):
        return self._retry(lambda: self.link.readline())

    def _retry(self, call):
        try:
            if self.link is None:
                self.reopen()
            return call()
        except OSError:
            self.reopen()
            return call()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
