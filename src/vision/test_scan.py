"""双相机回归测试，实拍样本仅保留棋盘区域。"""

from pathlib import Path
import unittest

import cv2
import numpy as np

from .board import fit_plane, project
from .field import Field
from .scan import inspect
from .tags import Detector


class DualCameraTests(unittest.TestCase):
    def test_printed_board_in_both_orientations(self):
        path = Path(__file__).with_name("fixtures") / "printed-board.jpg"
        frame = cv2.imread(str(path))
        detector = Detector()
        for turn in (0, 1):
            tags = detector.detect(np.rot90(frame, turn).copy())
            keys = {("printed5", i) for i in range(4)}
            self.assertEqual(set(tags), keys)
            plane = fit_plane(tags)
            self.assertEqual(set(plane.tags_used), keys)
            center = project(plane.to_pixel, [(100, 100)])[0]
            expected = (234, 246) if not turn else (246, frame.shape[1] - 1 - 234)
            np.testing.assert_allclose(center, expected, atol=7)

    def test_front_camera_only_reports_wall_tags(self):
        image = np.full((250, 450), 255, np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        for offset, ident in ((30, 0), (180, 1)):
            image[40:140, offset:offset+100] = cv2.aruco.generateImageMarker(dictionary, ident, 100)
        tags = Detector(("36h11",), board=False).detect(image)
        result = inspect(tags, "front", Field())
        self.assertEqual([t["id"] for t in result["tags"]], [1])
        np.testing.assert_allclose(result["tags"][0]["center_px"], (229.5, 89.5), atol=1)
        self.assertEqual(inspect({}, "front", Field())["tags"], [])
