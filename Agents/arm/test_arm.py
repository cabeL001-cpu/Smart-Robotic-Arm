import unittest

from . import ArmConfig, ServoCalibration, arm, cell_center, forward, inverse, joint_command


class ArmTests(unittest.TestCase):
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
        config = ArmConfig(elbow=ServoCalibration(zero_us=1490, us_per_degree=-7))
        self.assertEqual(joint_command((0, 10), config), b"ARM 1500 1420\n")
        with self.assertRaises(ValueError):
            joint_command((0, 140), config)

    def test_shared_link_only_sends_arm_frame(self):
        from unittest.mock import Mock
        link = Mock()
        link.readline.return_value = b"OK ARM\r\n"
        angles = arm("4", link=link)
        link.write.assert_called_once_with(joint_command(angles, ArmConfig()))
