"""按相机安装位置检查每帧中的标记，不访问底盘和舵机。"""

import argparse
import json

from .board import fit_plane
from .camera import Mount, frames
from .field import Field, WALLS
from .tags import Detector


def inspect(tags, role, field):
    if role == "front":
        tags = {k: q for k, q in tags.items() if k[0] == field.wall_family and k[1] in WALLS}
    result = {"camera": role, "tags": [
        {"family": family, "id": ident, "center_px": corners.mean(axis=0).tolist(),
         "corners_px": corners.tolist()} for (family, ident), corners in sorted(tags.items())
    ]}
    if role == "top":
        plane = fit_plane(tags)
        result["board_tags_used"] = len(plane.tags_used) if plane else 0
        result["board_rms_px"] = plane.rms_px if plane else None
    return result


def main():
    parser = argparse.ArgumentParser(description="检查前方墙标或顶摄棋盘、末端标记")
    parser.add_argument("camera", choices=("front", "top"))
    parser.add_argument("--source", required=True, help="相机编号、V4L2 设备路径、图片或录像")
    parser.add_argument("--config", help="该相机自己的安装参数 JSON")
    parser.add_argument("--field", help="场地配置，含墙标家族 wall_family")
    parser.add_argument("--frames", type=int, help="采样帧数，默认读完整个输入")
    args = parser.parse_args()
    if args.frames is not None and args.frames < 1:
        parser.error("--frames 至少为 1")
    mount, field = Mount.load(args.config), Field.load(args.field)
    top = args.camera == "top"
    detector = Detector(("36h11", mount.tip_family) if top else (field.wall_family,), board=top)
    for index, (t, frame) in enumerate(frames(args.source, mount), 1):
        result = inspect(detector.detect(frame), args.camera, field)
        print(json.dumps({"t": t, **result}, allow_nan=False), flush=True)
        if index == args.frames:
            break


if __name__ == "__main__":
    main()
