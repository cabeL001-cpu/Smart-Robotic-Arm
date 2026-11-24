import unittest
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import Mock

from . import ArmConfig, ServoCalibration, arm, cell_center, forward, inverse, joint_command


class ArmTests(unittest.TestCase):
    # 仅作可复现的仿真标定，两关节允许的几何区间不同。
    cfg = ArmConfig(shoulder=ServoCalibration(angle_at_max_deg=15),
                    elbow=ServoCalibration(angle_at_max_deg=130))

    def test_every_cell_round_trips_through_joint_limits(self):
        for cell in range(9):
            target = cell_center(cell)
            angles = inverse(*target)
            for wanted, measured in zip(target, forward(*angles)):
                self.assertAlmostEqual(wanted, measured, places=6)
            self.assertLessEqual(abs(angles[0]), 85)
            self.assertLessEqual(abs(angles[1]), 135)

    def test_unreachable_and_calibrated_pulses(self):
        with self.assertRaises(ValueError):
            inverse(1200, 0)
        config = replace(self.cfg, elbow=ServoCalibration(zero_us=1490, us_per_degree=-7,
                                                         angle_at_max_deg=40))
        self.assertEqual(joint_command((0, 10), config), b"ARM 1500 1420\n")
        with self.assertRaises(ValueError):
            joint_command((0, 140), config)

    def test_shared_link_only_sends_arm_frame(self):
        link = Mock()
        link.readline.return_value = b"OK ARM\r\n"
        angles = arm("4", config=self.cfg, link=link)
        link.write.assert_called_once_with(joint_command(angles, self.cfg))

    def test_last_90_degrees_both_mounting_directions_and_margin(self):
        servo = ServoCalibration(angle_at_max_deg=45, us_per_degree=-8, margin_deg=0)
        self.assertEqual(servo.angle_limits(135), [-45, 45])
        self.assertEqual(servo.pulse(-45), 1860)
        self.assertEqual(servo.pulse(45), 1140)
        for q in (-45.1, 45.1):
            with self.assertRaises(ValueError):
                servo.pulse(q)
        opposite = replace(servo, angle_at_max_deg=-45, retreat_sign=1, margin_deg=3)
        self.assertEqual(opposite.angle_limits(135), [-42, 42])
        trimmed = replace(opposite, min_us=1450, max_us=1550)
        self.assertEqual(trimmed.pulse_limits(135), (1450, 1550))

    def test_planner_uses_other_branch_or_refuses_before_sending(self):
        opposite = ArmConfig(shoulder=ServoCalibration(angle_at_max_deg=85),
                             elbow=ServoCalibration(angle_at_max_deg=-30))
        angles = opposite.solve(*cell_center(4), previous=(-58, 95))
        self.assertLess(angles[1], 0)
        for coord, wanted in zip(forward(*angles), cell_center(4)):
            self.assertAlmostEqual(coord, wanted)
        link = Mock()
        for cfg in (ArmConfig(), replace(opposite, elbow=self.cfg.elbow)):
            with self.assertRaises(ValueError):
                arm(4, config=cfg, link=link)
        link.write.assert_not_called()

    def test_firmware_limits_match_host_and_incomplete_export_preserves_file(self):
        with TemporaryDirectory() as folder:
            out = Path(folder) / "arm_limits.h"
            self.cfg.write_firmware_limits(out)
            contents = out.read_bytes()
            for name, (servo, cap) in zip(("SHOULDER", "ELBOW"), self.cfg.axes()):
                for end, pulse in zip(("MIN", "MAX"), servo.pulse_limits(cap)):
                    self.assertIn(f"{name}_{end}_US = {pulse};".encode(), contents)
            with self.assertRaises(ValueError):
                ArmConfig().write_firmware_limits(out)
            self.assertEqual(out.read_bytes(), contents)
