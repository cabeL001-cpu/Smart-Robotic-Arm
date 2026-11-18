"""二维惯性推算及绝对位姿校正。可单独注入带时间戳的样本。"""

from collections import deque
from dataclasses import dataclass
import math

from .sensor import Sample


G = 9.80665


def wrap(yaw):
    return math.atan2(math.sin(yaw), math.cos(yaw))


def rotate(x, y, a):
    c, s = math.cos(a), math.sin(a)
    return c*x - s*y, s*x + c*y


def attitude(q):
    norm = math.hypot(*q)
    if not math.isfinite(norm) or norm == 0:
        raise ValueError("无效的 AHRS 四元数")
    w, x, y, z = (v/norm for v in q)
    return ((1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)),
            (2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)),
            (2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)))


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


@dataclass(frozen=True)
class State:
    t: float
    x_m: float
    y_m: float
    yaw_rad: float
    vx_mps: float
    vy_mps: float


class Tracker:
    def __init__(self, stationary_samples, *, x_m=0., y_m=0., yaw_rad=0., mount_yaw_rad=0.):
        """在已知位置静止，校准机体系加速度零偏及 AHRS 航向与场地的夹角。"""
        samples = list(stationary_samples)
        if len(samples) < 2 or any(b.t <= a.t for a,b in zip(samples, samples[1:])):
            raise ValueError("静止校准需要连续递增的多个采样时刻")
        biases = []
        for s in samples:
            if math.hypot(*s.gyro_rad_s) > .12 or abs(math.hypot(*s.accel_g)-1) > .12:
                raise ValueError("校准期间检测到运动，请保持车身静止")
            r = attitude(s.quat)
            sign = 1 if dot(r[2], s.accel_g) >= 0 else -1
            biases.append([a-sign*z for a,z in zip(s.accel_g, r[2])])
        self.bias_g = tuple(sum(axis)/len(samples) for axis in zip(*biases))
        self.mount_yaw = mount_yaw_rad
        self.offset = yaw_rad + mount_yaw_rad - math.atan2(r[1][0], r[0][0])
        self.state = State(samples[-1].t, x_m, y_m, wrap(yaw_rad), 0., 0.)
        self.previous_acc = (0., 0.)
        self.history = deque([self.state])
        self.last_fix = float("-inf")

    def update(self, sample, stationary=False):
        """stationary 由已知停车状态传入。匀速行驶时保留已有速度。"""
        old = self.state
        dt = sample.t-old.t
        if dt <= 0:
            raise ValueError("样本时间必须递增")
        r = attitude(sample.quat)
        yaw = wrap(math.atan2(r[1][0], r[0][0]) + self.offset - self.mount_yaw)
        body = tuple((a-b)*G for a,b in zip(sample.accel_g, self.bias_g))
        # 先三维旋转，再取水平分量。重力在导航 Z 轴，无需在机体 XY 上硬减常数。
        ax, ay = rotate(dot(r[0], body), dot(r[1], body), self.offset)
        if math.hypot(ax, ay) < .035:
            ax = ay = 0.
        if stationary or dt > .3:
            vx = vy = ax = ay = 0.
            x, y = old.x_m, old.y_m
            if dt > .3:
                self.history.clear()
        else:
            vx = old.vx_mps + (self.previous_acc[0]+ax)*dt/2
            vy = old.vy_mps + (self.previous_acc[1]+ay)*dt/2
            x = old.x_m + (old.vx_mps+vx)*dt/2
            y = old.y_m + (old.vy_mps+vy)*dt/2
        self.previous_acc = ax, ay
        self.state = State(sample.t, x, y, yaw, vx, vy)
        self.history.append(self.state)
        while len(self.history)>1 and self.history[1].t < sample.t-1:
            self.history.popleft()
        return self.state

    def correct(self, t, x_m, y_m, yaw_rad):
        """将绝对观测与对应时刻的估计对齐，并保留曝光后发生的相对运动。"""
        if t <= self.last_fix or not self.history[0].t <= t <= self.state.t or self.state.t-t > 1:
            return False
        before, after = self.history[0], self.state
        for s in self.history:
            if s.t >= t:
                after = s
                break
            before = s
        f = (t-before.t)/(after.t-before.t) if after.t > before.t else 0.
        ref_x = before.x_m+f*(after.x_m-before.x_m)
        ref_y = before.y_m+f*(after.y_m-before.y_m)
        ref_yaw = before.yaw_rad+f*wrap(after.yaw_rad-before.yaw_rad)
        turn = wrap(yaw_rad-ref_yaw)
        self.offset = wrap(self.offset+turn)
        self.previous_acc = rotate(*self.previous_acc, turn)
        corrected = deque()
        for s in self.history:
            dx, dy = rotate(s.x_m-ref_x, s.y_m-ref_y, turn)
            vx, vy = rotate(s.vx_mps, s.vy_mps, turn)
            corrected.append(State(s.t, x_m+dx, y_m+dy, wrap(s.yaw_rad+turn), vx, vy))
        self.history, self.state, self.last_fix = corrected, corrected[-1], t
        return True
