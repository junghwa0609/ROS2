# Copyright 2025 JungHwa Lee
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import unittest

from safety_node.control import closest_front_distance, target_speed


class ClosestFrontDistanceTests(unittest.TestCase):
    def test_uses_only_valid_readings_in_the_front_sector(self):
        distance = closest_front_distance(
            scan_ranges=[0.1, 1.2, 0.4, float('nan'), 0.2],
            angle_min=-1.0,
            angle_increment=0.5,
            front_half_angle_rad=0.6,
            minimum_valid_distance_m=0.05,
        )

        self.assertEqual(distance, 0.4)

    def test_returns_none_without_a_valid_front_reading(self):
        distance = closest_front_distance(
            scan_ranges=[1.0, float('inf'), float('nan'), 0.01, 1.0],
            angle_min=-1.0,
            angle_increment=0.5,
            front_half_angle_rad=0.6,
            minimum_valid_distance_m=0.05,
        )

        self.assertIsNone(distance)

    def test_returns_none_for_an_empty_scan(self):
        self.assertIsNone(closest_front_distance([], -1.0, 0.1, 0.5, 0.05))


class TargetSpeedTests(unittest.TestCase):
    def test_stops_when_data_is_unavailable_or_obstacle_is_close(self):
        arguments = (0.5, 1.5, 0.5, 1.5)

        self.assertEqual(target_speed(None, *arguments), 0.0)
        self.assertEqual(target_speed(0.49, *arguments), 0.0)

    def test_uses_slow_and_normal_speed_zones(self):
        arguments = (0.5, 1.5, 0.5, 1.5)

        self.assertEqual(target_speed(0.5, *arguments), 0.5)
        self.assertEqual(target_speed(1.49, *arguments), 0.5)
        self.assertEqual(target_speed(1.5, *arguments), 1.5)

    def test_rejects_invalid_thresholds(self):
        with self.assertRaises(ValueError):
            target_speed(1.0, 1.5, 0.5, 0.5, 1.5)


if __name__ == '__main__':
    unittest.main()
