import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from .board_route import BoardApproach
from .imu import rotate, wrap
from .nav_config import Control, Origin
from .navigation import Navigator


class SimulatedNavigator(Navigator):
    def __init__(self, start, yaw, heading=90):
        config = SimpleNamespace(
            board_approach=BoardApproach(work_distance_m=.40, heading_deg=heading),
            control=Control(tolerance_m=.03),
            speeds={"go": .16, "back": .15, "left": .14, "right": .14,
                    "rotate-left": 65., "rotate-right": 65.},
            origin=Origin(0., 0., 0.),
        )
        field = SimpleNamespace(bounds_m=(0, 0, 2, 2), board_origin_mm=(900, 900), board_yaw_deg=0)
        self.state = SimpleNamespace(x_m=start[0], y_m=start[1], yaw_rad=math.radians(yaw))
        self.pulses = []
        imu = SimpleNamespace(snapshot=lambda: (self.state, 0.))
        super().__init__(config, field, imu, None, None, None)
        self.area = self.board.travel

    def _check(self, deadline):
        if len(self.pulses) >= 250:
            raise AssertionError("模拟运动未收敛")

    def _pulse(self, action, amount, deadline, *, limit_ms=None, reference=None):
        self._check(deadline)
        speed = self.config.speeds[action]
        ms = max(self.control.min_pulse_ms, min(self.control.max_pulse_ms, round(1000 * amount / speed)))
        if limit_ms is not None:
            ms = min(ms, limit_ms)
        delta = ms * speed / 1000
        if action.startswith("rotate"):
            sign = 1 if action == "rotate-left" else -1
            self.state.yaw_rad = wrap(self.state.yaw_rad + math.radians(delta) * sign)
        else:
            axes = {"go": (1, 0), "back": (-1, 0), "left": (0, 1), "right": (0, -1)}
            ahead, left = axes[action]
            dx, dy = rotate(ahead * delta, left * delta, self.state.yaw_rad)
            old = self.state.x_m, self.state.y_m
            new = old[0] + dx, old[1] + dy
            if not self.area.allows(old, new):
                raise AssertionError(f"运动越出预留净空：{old} -> {new}")
            self.state.x_m, self.state.y_m = new
        self.pulses.append(action)

    def _observe(self, deadline):
        return 1., self.state, self.state

    def _fix_at_rest(self, origin, deadline, *, observe=None):
        s = self.state
        return [SimpleNamespace(x_m=s.x_m, y_m=s.y_m, yaw_rad=s.yaw_rad, tag_ids=(0, 1))] * 3


class BoardRouteTests(unittest.TestCase):
    def test_crossing_path_goes_around_board(self):
        nav = SimulatedNavigator((1., 1.6), 0)
        start = 1., 1.6
        route = nav.board.travel.path(start, nav.board.staging, .055)
        self.assertFalse(nav.board.travel.allows(start, nav.board.staging))
        self.assertGreater(len(route), 1)
        self.assertTrue(all(nav.board.travel.allows(a, b) for a, b in zip([start, *route], route)))

    def test_field_goal_on_board_is_rejected_before_motion(self):
        nav = SimulatedNavigator((1., .50), 90)
        with self.assertRaises(ValueError):
            nav.go_to(1., 1., frame="field")
        self.assertEqual(nav.pulses, [])

    def test_failed_board_confirmation_does_not_start_another_pulse(self):
        nav = SimulatedNavigator((1., .50), 90)
        nav._fix_at_rest = Mock(side_effect=TimeoutError)
        with self.assertRaises(TimeoutError):
            nav._board_lane(nav.board.work, 1, observe=nav._observe)
        self.assertEqual(nav.pulses, [])

    def test_route_approach_and_retreat(self):
        for heading in (0, 90, 180):
            for yaw in range(0, 360, 15):
                for start in ((1., 1.60), (.40, 1.), (1.60, 1.60), (.51, .51)):
                    with self.subTest(start=start, yaw=yaw, heading=heading):
                        nav = SimulatedNavigator(start, yaw, heading)
                        nav._approach(nav.board.staging, 1)
                        while math.dist((nav.state.x_m, nav.state.y_m), nav.board.staging) > .03:
                            target, _ = nav._travel_target(nav.board.staging)
                            nav._translate(target, .55, 1, area=nav.area)
                        nav._align_for_board(1)
                        nav.area = nav.board.near
                        arrival = nav._board_lane(nav.board.work, 1, observe=nav._observe)
                        self.assertLessEqual(arrival.error_m, .015)
                        nav._retreat_from_board(1)
