"""实时传感器适配。串口与相机各有单独读线程，Uno 指令只在导航线程发送。"""

from contextlib import closing, contextmanager
import math
from threading import Condition, Event, Thread
from time import monotonic

from .imu import Tracker, wrap
from .imu.sensor import serial_samples

if "." in __package__:
    from ..vision.camera import frames, live_source
else:
    from vision.camera import frames, live_source


class _Reader:
    def __init__(self, name):
        self.name = name
        self.cv = Condition()
        self.closing = Event()
        self.error = None

    @contextmanager
    def running(self, read_loop):
        def run():
            try:
                read_loop()
                if not self.closing.is_set():
                    raise OSError(f"{self.name} 数据流结束")
            except Exception as error:
                with self.cv:
                    self.error = error
                    self.cv.notify_all()

        worker = Thread(target=run, name=self.name, daemon=True)
        worker.start()
        try:
            yield self
        finally:
            self.closing.set()
            worker.join(timeout=3.5)

    def check(self):
        if self.error is not None:
            raise OSError(f"{self.name} 读取失败：{self.error}") from self.error

    def wait(self, predicate, deadline):
        # 调用方持有条件锁。线程故障也要唤醒正在等待新数据的控制器。
        ready = self.cv.wait_for(lambda: self.error is not None or predicate(),
                                 timeout=max(0., deadline - monotonic()))
        self.check()
        if not ready or monotonic() > deadline:
            raise TimeoutError(f"等待 {self.name} 新数据超时")


class ImuFeed(_Reader):
    def __init__(self, tracker):
        super().__init__("IMU")
        self.tracker = tracker
        self.turn = 0.               # AHRS 相邻采样的有符号转角，不计入视觉修正
        self.gyro_turn = 0.          # 陀螺竖直分量积分，不经过磁力计
        self.last_rate = None
        self.rest_from = 0.

    def consume(self, samples):
        for sample in samples:
            if self.closing.is_set():
                return
            with self.cv:
                before = self.tracker.state
                if sample.t - before.t > .3:
                    raise TimeoutError("IMU 中断超过 0.3 秒，无法连续推算")
                rate = self.tracker.turn_rate(sample)
                dt = sample.t - before.t
                now = self.tracker.update(sample, stationary=sample.t >= self.rest_from)
                self.turn += wrap(now.yaw_rad - before.yaw_rad)
                # 实车磁力计受电机和电池干扰，改用陀螺梯形积分累计转角。
                if self.last_rate is not None:
                    self.gyro_turn += (self.last_rate + rate) * dt / 2
                self.last_rate = rate
                self.cv.notify_all()

    def snapshot(self):
        with self.cv:
            self.check()
            state = self.tracker.state
            if monotonic() - state.t > .3:
                raise TimeoutError("IMU 位姿过期")
            return state, self.turn

    def snapshot_gyro(self):
        """与 snapshot 相同，但累计转角来自陀螺积分，不受磁力计航向漂移影响。"""
        with self.cv:
            self.check()
            state = self.tracker.state
            if monotonic() - state.t > .3:
                raise TimeoutError("IMU 位姿过期")
            return state, self.gyro_turn

    def moving(self):
        with self.cv:
            self.rest_from = math.inf

    def settle(self, seconds, deadline):
        with self.cv:
            self.rest_from = monotonic() + seconds
            self.wait(lambda: self.tracker.state.t >= self.rest_from, deadline)
            return self.rest_from

    def at(self, stamp, deadline):
        with self.cv:
            self.wait(lambda: self.tracker.state.t >= stamp, deadline)
            state = self.tracker.state_at(stamp)
            if state is None:
                raise TimeoutError("相机帧已超出 IMU 可对齐的一秒历史")
            return state

    def correct(self, stamp, x, y, yaw):
        with self.cv:
            if not self.tracker.correct(stamp, x, y, yaw):
                raise TimeoutError("前摄修正已过期或时间戳重复")


@contextmanager
def open_imu(port, calibration_s, mount_yaw_deg):
    import serial
    with serial.Serial(port, 115200, timeout=.1, exclusive=True) as link:
        link.reset_input_buffer()
        samples = serial_samples(link)
        stationary = []
        deadline = monotonic() + calibration_s + 3.
        for sample in samples:
            if monotonic() > deadline or (stationary and sample.t - stationary[-1].t > .3):
                raise TimeoutError("IMU 静止标定采样不连续")
            stationary.append(sample)
            if sample.t - stationary[0].t >= calibration_s:
                break
        tracker = Tracker(stationary, mount_yaw_rad=math.radians(mount_yaw_deg))
        reader = ImuFeed(tracker)
        with reader.running(lambda: reader.consume(samples)):
            yield reader


class CameraFrames(_Reader):
    def __init__(self, name):
        super().__init__(name)
        self.latest = None

    def capture(self, source, mount):
        with closing(frames(source, mount)) as stream:
            for stamp, frame in stream:
                if self.closing.is_set():
                    return
                with self.cv:
                    self.latest = stamp, frame
                    self.cv.notify_all()

    def after(self, stamp, deadline):
        with self.cv:
            self.wait(lambda: self.latest is not None and self.latest[0] > stamp
                      and monotonic() - self.latest[0] < .5, deadline)
            return self.latest

    def check_live(self):
        with self.cv:
            self.check()
            if self.latest is None or monotonic() - self.latest[0] >= .5:
                raise TimeoutError(f"{self.name}采集中断或帧已过期")


@contextmanager
def open_camera(source, mount, name="前摄"):
    if not live_source(source):
        raise ValueError(f"实际导航须使用实时{name}设备")
    reader = CameraFrames(name)
    with reader.running(lambda: reader.capture(source, mount)):
        yield reader


open_front = open_camera
