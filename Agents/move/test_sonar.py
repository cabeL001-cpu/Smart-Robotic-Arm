import unittest
from unittest.mock import Mock

from . import MotionBlocked, move
from .sonar import read, scan


class SonarLinkTests(unittest.TestCase):
    def test_scan_and_snapshot_on_one_link(self):
        link = Mock()
        link.readline.side_effect = (b"OK STOP\n", b"SCAN 0x69 0x6C 0x6A\n",
                                     b"SONAR 0x69=301 0x72=NA 0x6C=1280 0x6A=10\n")
        self.assertEqual(scan(link), [0x69, 0x6C, 0x6A])
        snapshot = read(link)
        self.assertEqual(snapshot["front"], {"address": "0x69", "mm": 301})
        self.assertEqual(snapshot["back"], {"address": "0x72", "mm": None})
        self.assertEqual(snapshot["left"], {"address": "0x6C", "mm": 1280})
        self.assertEqual(snapshot["right"], {"address": "0x6A", "mm": 10})

    def test_motion_reports_rejection_and_emergency_stop_without_restarting(self):
        for replies in ((b"ERR SONAR FRONT\n",), (b"OK MOVE\n", b"ERR OBSTACLE FRONT 250\n")):
            with self.subTest(replies=replies):
                link = Mock()
                link.readline.side_effect = replies
                with self.assertRaisesRegex(MotionBlocked, replies[-1].decode().strip()):
                    move("go", 800, link=link)
                link.write.assert_called_once_with(b"GO 800\n")
