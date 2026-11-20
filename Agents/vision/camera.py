"""一台向下相机的输入与安装参数。像素坐标使用去畸变后的原分辨率。"""

from dataclasses import dataclass, asdict
import json
from pathlib import Path
from time import monotonic
import sys

import cv2
import numpy as np


@dataclass
class Mount:
    image_size: tuple = ()         # 宽、高，已标定的像素参数须在此分辨率下使用
    camera_matrix: tuple = ()
    distortion: tuple = ()
    height_mm: float = 1500.0      # 镜头中心高出棋盘标记平面的实测距离
    nadir_px: tuple = ()           # 镜头铅垂线与棋盘平面交点对应的像素
    tip_height_mm: float = 0.0
    tip_offset_mm: tuple = (0.0, 0.0)  # 标签自身右、上方向的指尖偏移
    tip_family: str = "36h10"
    tip_id: int = 100
    arm_base_px: tuple = ()        # 基座轴在棋盘平面的投影
    arm_x_px: tuple = ()           # 沿机械臂零角方向的第二个平面点

    robot_center_px: tuple = ()     # 车体中心的地面投影像素
    robot_forward_px: tuple = ()    # 前方第二个地面点的像素

    @classmethod
    def load(cls, path=None):
        return cls(**json.loads(Path(path).read_text(encoding="utf-8"))) if path else cls()

    def save(self, path):
        Path(path).write_text(json.dumps(asdict(self), indent=2) + "\n", encoding="utf-8")

    def rectify(self, frame):
        size = frame.shape[1], frame.shape[0]
        if self.image_size and tuple(self.image_size) != size:
            raise ValueError(f"输入分辨率 {size} 与标定 {self.image_size} 不一致")
        if self.camera_matrix:
            return cv2.undistort(frame, np.array(self.camera_matrix, dtype=float),
                                 np.array(self.distortion, dtype=float) if self.distortion else None)
        return frame


def live_source(source):
    if str(source).isdecimal():
        return True
    path = Path(source).resolve()
    return path.parent == Path("/dev") and path.name.startswith("video") and path.name[5:].isdecimal() and path.is_char_device()


def frames(source, mount):
    """相机用单调时钟，视频用录像时间。静态图片只产生一帧。"""
    if not str(source).isdecimal() and Path(source).suffix.lower() in {".png", ".jpg", ".jpeg", ".bmp"}:
        frame = cv2.imread(str(source))
        if frame is None:
            raise OSError(f"图片读取失败 {source}")
        yield 0.0, mount.rectify(frame)
        return
    live = live_source(source)
    device = int(source) if str(source).isdecimal() else str(source)
    backend = cv2.CAP_V4L2 if live and sys.platform.startswith("linux") else cv2.CAP_ANY
    capture = cv2.VideoCapture(device, backend)
    try:
        if not capture.isOpened():
            raise OSError(f"无法打开相机或录像 {source}")
        if live and sys.platform.startswith("linux"):
            # 两路 USB 摄像头共享总线时优先请求压缩帧；驱动可保留自己的默认格式。
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        if live and mount.image_size:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, mount.image_size[0])
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, mount.image_size[1])
        while True:
            ok, frame = capture.read()
            if not ok:
                if live:
                    raise OSError("相机帧读取中断")
                return
            t = monotonic() if live else capture.get(cv2.CAP_PROP_POS_MSEC) / 1000
            yield t, mount.rectify(frame)
    finally:
        capture.release()
