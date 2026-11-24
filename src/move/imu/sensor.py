"""YbImu 二进制串口及原始数据回放。加速度用 g，角速度用 rad/s。"""

import csv
from dataclasses import dataclass
import math
import struct
from time import monotonic


@dataclass(frozen=True)
class Sample:
    t: float
    accel_g: tuple
    gyro_rad_s: tuple
    quat: tuple  # w, x, y, z，传感器坐标到 AHRS 导航坐标


HEADER = b"\x7e\x23"
COLUMNS = "time_s ax_g ay_g az_g gx_rad_s gy_rad_s gz_rad_s qw qx qy qz".split()


def euler_quaternion(roll, pitch, yaw):
    cr, cp, cy = (math.cos(a/2) for a in (roll, pitch, yaw))
    sr, sp, sy = (math.sin(a/2) for a in (roll, pitch, yaw))
    return (cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy)


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.raw = None

    def feed(self, data, received_at):
        """按总长度和累加和重同步。每个原始数据帧只与一组新姿态配对。"""
        self.buffer.extend(data)
        while len(self.buffer) >= 3:
            start = self.buffer.find(HEADER)
            if start < 0:
                self.buffer[:] = HEADER[:1] if self.buffer.endswith(HEADER[:1]) else b""
                break
            del self.buffer[:start]
            if len(self.buffer) < 3:
                break
            n = self.buffer[2]
            if not 5 <= n <= 40:
                del self.buffer[0]
                continue
            if len(self.buffer) < n:
                break
            packet = self.buffer[:n]
            if (sum(packet[:-1]) & 255) != packet[-1]:
                del self.buffer[0]
                continue
            del self.buffer[:n]
            # 无设备时间戳，按同批剩余字节的 8N1 线速估计接收时刻。
            t = received_at - len(self.buffer) * 10 / 115200
            kind, payload = packet[3], packet[4:-1]
            if kind == 0x04 and len(payload) == 18:
                raw = struct.unpack("<9h", payload)
                accel = tuple(v * 16/32767 for v in raw[:3])
                gyro = tuple(math.radians(v * 2000/32767) for v in raw[3:6])
                self.raw = t, accel, gyro
            elif self.raw is not None:
                if kind == 0x16 and len(payload) == 16:
                    q = struct.unpack("<4f", payload)
                elif kind == 0x26 and len(payload) == 12:
                    q = euler_quaternion(*struct.unpack("<3f", payload))
                else:
                    continue
                stamp, accel, gyro = self.raw
                self.raw = None
                if 0 <= t-stamp < .15 and .5 < math.hypot(*q) < 1.5:
                    yield Sample(stamp, accel, gyro, q)


def serial_samples(link):
    decoder, last = Decoder(), monotonic()
    while True:
        data = link.read(1)
        data += link.read(link.in_waiting)
        now = monotonic()
        for sample in decoder.feed(data, now):
            last = now
            yield sample
        if now-last > 3:
            raise TimeoutError("三秒内未收到有效的原始数据与姿态，请核对 YbImu 串口和输出设置")


def read_csv(stream):
    reader = csv.DictReader(stream)
    if not set(COLUMNS).issubset(reader.fieldnames or ()):
        raise ValueError("回放需要原始加速度、角速度和四元数。轨迹结果 CSV 仅含计算后的位置")
    for row in reader:
        v = [float(row[k]) for k in COLUMNS]
        if not all(map(math.isfinite, v)):
            raise ValueError("样本含非有限数值")
        yield Sample(v[0], tuple(v[1:4]), tuple(v[4:7]), tuple(v[7:11]))


def record_samples(samples, stream):
    writer = csv.writer(stream)
    writer.writerow(COLUMNS)
    for s in samples:
        writer.writerow([s.t, *s.accel_g, *s.gyro_rad_s, *s.quat])
        yield s
