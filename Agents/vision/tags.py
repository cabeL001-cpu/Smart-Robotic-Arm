"""按 (家族, ID) 区分标签。四个角点从码图左上开始顺时针排列。"""

import cv2
import numpy as np

from .printed import printed_code, recover_neighbors


class Detector:
    def __init__(self, families=("36h11", "36h10"), *, board=True):
        self.board = board
        self.detectors = {}
        self.dictionaries = {}
        for family in dict.fromkeys(families):
            dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, "DICT_APRILTAG_" + family))
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detectors[family] = cv2.aruco.ArucoDetector(dictionary, params)
            self.dictionaries[family] = dictionary

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        tags = {}
        for family, detector in self.detectors.items():
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None:
                tags.update({(family, int(i)): q.reshape(4, 2) for i, q in zip(ids.ravel(), corners)})
        if self.board and "36h11" in self.detectors and any(("36h11", i) not in tags for i in range(4)):
            self._board_edges(gray, tags)
            recover_neighbors(gray, tags)
        return tags

    def _board_edges(self, gray, tags):
        # 印刷模板的格线连着标签黑边。断开细格线后找候选四边形，仍从原图解码。
        binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY_INV, 51, 8)
        dst = np.float32([[0, 0], [79, 0], [79, 79], [0, 79]])
        for width, join_cells in ((5, True), (7, False), (3, True), (9, True), (13, True)):
            kernel = np.ones((width, width), np.uint8)
            solid = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel) if join_cells else binary
            blobs = cv2.morphologyEx(solid, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(blobs, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if not 120 < cv2.contourArea(contour) < gray.size * 0.08:
                    continue
                contour = cv2.convexHull(contour)
                q = cv2.approxPolyDP(contour, 0.045 * cv2.arcLength(contour, True), True)
                if len(q) != 4 or not cv2.isContourConvex(q):
                    continue
                q = q.reshape(4, 2).astype(np.float32)
                if cv2.contourArea(q, oriented=True) < 0:
                    q = q[::-1].copy()
                q = np.roll(q, -np.argmin(q.sum(axis=1)), axis=0)
                square = cv2.warpPerspective(gray, cv2.getPerspectiveTransform(q, dst), (80, 80))
                _, bits = cv2.threshold(square, 0, 1, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
                printed = printed_code(bits)
                if printed is not None:
                    ident, turn = printed
                    tags.setdefault(("printed5", ident), np.roll(q, turn, axis=0))
                    continue
                cells = (bits.reshape(8, 10, 8, 10)[:, 3:7, :, 3:7].mean(axis=(1, 3)) > 0.5).astype(np.uint8)
                border = np.r_[cells[0], cells[-1], cells[1:-1, 0], cells[1:-1, -1]]
                if border.sum() > 4:
                    continue
                valid, tag_id, rotation = self.dictionaries["36h11"].identify(cells[1:-1, 1:-1].copy(), 0)
                if valid and tag_id in range(4):
                    tags.setdefault(("36h11", tag_id), np.roll(q, rotation, axis=0))
            if any(all((family, i) in tags for i in range(4)) for family in ("printed5", "36h11")):
                return
