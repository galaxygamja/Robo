from __future__ import annotations

import math
import random
import unittest

from robo_control.collision_geometry import (
    segment_near_rectangle,
    segment_rectangle_distance_sq,
)
from robo_control.control_loop import _polygon_distance


class RectangleCollisionTests(unittest.TestCase):
    def test_inside_crossing_touch_and_degenerate_segments(self):
        box = (10., 20., 30., 50.)
        for start, end, expected in (((0, 30), (40, 30), 0), ((20, 30), (20, 30), 0),
                                    ((0, 20), (10, 20), 0), ((0, 10), (40, 10), 100),
                                    ((0, 10), (0, 10), 200), ((40, 60), (40, 60), 200),
                                    ((20, 60), (20, 80), 100), ((0, 0), (0, 100), 100)):
            with self.subTest(start=start, end=end):
                self.assertAlmostEqual(expected, segment_rectangle_distance_sq(start, end, box))

    def test_corners_are_round_not_overinflated_squares(self):
        box = (10., 10., 20., 20.)
        self.assertFalse(segment_near_rectangle((0, 0), (0, 0), box, 10.))
        self.assertTrue(segment_near_rectangle((0, 10), (0, 10), box, 10.))
        self.assertTrue(segment_near_rectangle((0, 0), (0, 0), box, math.sqrt(200)))

    def test_deterministic_random_segments_match_existing_general_polygon_oracle(self):
        rng = random.Random(20260907)
        for index in range(6000):
            x, y = rng.uniform(-500, 1500), rng.uniform(-500, 1500)
            w, h = rng.uniform(.01, 500), rng.uniform(.01, 500)
            start = (rng.uniform(-500, 1500), rng.uniform(-500, 1500))
            end = start if index % 7 == 0 else (rng.uniform(-500, 1500), rng.uniform(-500, 1500))
            box = (x, y, x+w, y+h)
            polygon = [(x, y), (x+w, y), (x+w, y+h), (x, y+h)]
            distance = _polygon_distance([start, end], polygon)
            actual = segment_rectangle_distance_sq(start, end, box)
            self.assertAlmostEqual(distance, math.sqrt(actual), delta=1e-6)
            clearance = rng.uniform(0, 300)
            self.assertEqual(distance <= clearance, segment_near_rectangle(start, end, box, clearance))
            if distance > 1e-6:
                self.assertFalse(segment_near_rectangle(start, end, box, distance-1e-6))
            self.assertTrue(segment_near_rectangle(start, end, box, distance))

    def test_direction_and_rectangle_translation_do_not_change_result(self):
        start, end, box = (1., 2.), (90., 30.), (60., 60., 90., 100.)
        expected = segment_rectangle_distance_sq(start, end, box)
        self.assertEqual(expected, segment_rectangle_distance_sq(end, start, box))
        moved = tuple(v+10000 for v in box)
        self.assertAlmostEqual(expected, segment_rectangle_distance_sq(tuple(v+10000 for v in start),
                                                                      tuple(v+10000 for v in end), moved))


if __name__ == "__main__":
    unittest.main()
