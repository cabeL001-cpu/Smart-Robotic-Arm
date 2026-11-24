import math
from pathlib import Path
import unittest

import cv2
import numpy as np

from .align import align, ArmConfig
from .arm import Hold, measure, target_xy
from .board import BOARD, fit_plane, project, square
from .camera import Mount
from .tags import Detector


class ArmVisionTests(unittest.TestCase):
    def setUp(self):
        self.h = np.array([[2.2, .12, 60], [.1, -2, 560], [.0002, .0001, 1.]])
        self.tags = {k: project(self.h, v) for k, v in BOARD.items()}
        self.tags["36h10", 100] = project(self.h, square(100, 100, 14))
        self.plane = fit_plane(self.tags)

    def test_detector_on_mixed_families_and_printed_template(self):
        H = np.array([[2., 0., 100.], [0., -2., 500.], [0., 0., 1.]])
        img = np.full((600, 600), 255, np.uint8)
        for (family, i), xy in {**BOARD, ("36h10", 100): square(100, 100, 20)}.items():
            uv = project(H, xy).round().astype(int)
            x, y = uv[0]
            dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, "DICT_APRILTAG_" + family))
            img[y:y+40, x:x+40] = cv2.aruco.generateImageMarker(dictionary, i, 40)
        detector = Detector()
        found = detector.detect(img)
        obs = measure(found, fit_plane(found), (600, 600), 4, Mount())
        self.assertLess(obs.error_mm, 1)
        slide = Path(__file__).with_name("fixtures") / "board.png"
        self.assertEqual(len(fit_plane(detector.detect(cv2.imread(str(slide)))).tags_used), 4)

    def test_parallax_millimetres_and_missing_observations(self):
        foot = project(self.h, [(0, 0)])[0].tolist()
        mount = Mount(nadir_px=foot, height_mm=1500, tip_height_mm=150)
        obs = measure(self.tags, self.plane, (640, 600), 4, mount)
        np.testing.assert_allclose(obs.tip_mm, (90, 90), atol=1e-4)
        self.assertAlmostEqual(obs.error_mm, math.sqrt(200), places=4)
        self.assertIsNone(measure({}, self.plane, (640, 600), 4, mount))
        self.assertIsNone(fit_plane({("36h11", 0): self.tags["36h11", 0]}))

    def test_hold_resets_on_lost_frame(self):
        obs = measure(self.tags, self.plane, (640, 600), 4, Mount())
        hold = Hold()
        for t in (0, .5, 1., 1.5):
            self.assertFalse(hold.update(t, obs))
        self.assertFalse(hold.update(2., None))
        for t in (2.5, 3., 3.5, 4.):
            self.assertFalse(hold.update(t, obs))
        self.assertTrue(hold.update(4.5, obs))

    def test_closed_loop_with_camera_moving_between_docking_poses(self):
        from dataclasses import replace
        base, ref = project(self.h, [(-150, 100), (-50, 100)]).tolist()
        mount = Mount(arm_base_px=base, arm_x_px=ref)
        sent = []
        def samples():
            for i in range(10):
                center = target_xy(4) if sent else (80, 80)
                self.tags["36h10", 100] = project(self.h, square(*center, 14))
                yield i * .4, self.plane, measure(self.tags, self.plane, (640, 600), 4, mount)
        cfg = ArmConfig()
        cfg = replace(cfg, shoulder=replace(cfg.shoulder, angle_at_max_deg=15),
                      elbow=replace(cfg.elbow, angle_at_max_deg=130))
        final = align(4, samples(), sent.append, mount, cfg)
        self.assertLess(final.error_mm, .001)
        self.assertEqual(len(sent), 1)
        nominal = replace(cfg.geometry, base_x_mm=-150, base_y_mm=100)
        np.testing.assert_allclose(sent[0], cfg.solve(100, 100, geometry=nominal), atol=1e-4)
        sent.clear()
        with self.assertRaises(ValueError):
            align(4, samples(), sent.append, mount, replace(cfg, shoulder=cfg.elbow))
        self.assertFalse(sent)
