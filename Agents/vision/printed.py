"""实物棋盘的四种印刷角标。编号按棋盘左下、右下、左上、右上排列。"""

import cv2
import numpy as np


# 以数字正立的棋盘为基准，1 表示白色。每个码有 5×5 个数据格和约半格黑边。
_PATTERNS = (
    "01001 10100 00010 11001 00110",
    "00101 11010 00100 10010 01001",
    "01010 10011 00100 11010 01001",
    "10010 01001 00100 10010 01010",
)


def _rotations():
    lookup = {}
    for ident, pattern in enumerate(_PATTERNS):
        bits = np.array(list(pattern.replace(" ", "")), dtype=np.uint8).reshape(5, 5)
        for turn in range(4):
            rotated = np.rot90(bits, turn).copy()
            lookup[rotated.tobytes()] = ident, turn
            # 四组码的旋转版本之间最少差四位，单比特纠错仍有唯一解。
            for index in range(rotated.size):
                rotated.flat[index] ^= 1
                lookup[rotated.tobytes()] = ident, turn
                rotated.flat[index] ^= 1
    return lookup


_CODES = _rotations()


def printed_code(bits):
    # 透视校正后的 80×80 方块。只取每个数据格中心，避开印刷和插值边缘。
    centers = np.rint(np.arange(1, 6) * 79 / 6).astype(int)
    cells = cv2.blur(bits.astype(np.float32), (5, 5))[np.ix_(centers, centers)] > 0.5
    border = np.r_[bits[3, centers], bits[-4, centers], bits[centers, 3], bits[centers, -4]]
    if border.sum() > 2:
        return None
    return _CODES.get(cells.astype(np.uint8).tobytes())

def recover_neighbors(gray, tags):
    """用已解出的两个实物角标缩小搜索区域，在原图上复核其余码字。"""
    from .board import BOARD_LAYOUT

    keys = [("printed5", i) for i in range(4)]
    seen = [k for k in keys if k in tags]
    if len(seen) not in (2, 3):
        return
    # 将棋盘正立图归一化到每毫米 4 像素，给角标外沿留出搜索余量。
    def pixels(key):
        return np.float32(BOARD_LAYOUT[key] * (4, -4) + (80, 880))

    world = np.vstack([pixels(k) for k in seen])
    image = np.vstack([tags[k] for k in seen])
    affine, _ = cv2.estimateAffinePartial2D(image, world)
    if affine is None:
        return
    back = cv2.invertAffineTransform(affine)
    for key in keys:
        if key in tags:
            continue
        origin = pixels(key)[0] - 60
        transform = affine.copy()
        transform[:, 2] -= origin
        region = cv2.warpAffine(gray, transform, (180, 180), borderValue=255)
        # 黑框约半格宽，四周各 5 像素，内部每格 10 像素。
        data = np.array(list(_PATTERNS[key[1]].replace(" ", "")), np.uint8).reshape(5, 5)
        template = np.pad(cv2.resize(data * 255, (50, 50), interpolation=cv2.INTER_NEAREST), 5)
        scores = cv2.matchTemplate(region, template, cv2.TM_CCOEFF_NORMED)
        _, score, _, (x, y) = cv2.minMaxLoc(scores)
        if score < 0.55:
            continue
        quad = np.float32([[x, y], [x+59, y], [x+59, y+59], [x, y+59]]) + origin
        corners = cv2.transform(quad[None], back)[0]
        if corners.min() < 0 or (corners >= gray.shape[::-1]).any():
            continue
        side = np.linalg.norm(corners[0] - corners[1])
        window = max(2, min(10, round(side * .08)))
        refined = corners.copy()
        cv2.cornerSubPix(gray, refined, (window, window), (-1, -1), (3, 15, .05))
        dst = np.float32([[0, 0], [79, 0], [79, 79], [0, 79]])
        for candidate in (refined, corners):
            patch = cv2.warpPerspective(gray, cv2.getPerspectiveTransform(candidate, dst), (80, 80))
            _, bits = cv2.threshold(patch, 0, 1, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            if printed_code(bits) == (key[1], 0):
                tags[key] = candidate
                break
