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

"""Pure helpers for selecting LiDAR readings and determining a safe speed."""

import math


def closest_front_distance(
    scan_ranges,
    angle_min,
    angle_increment,
    front_half_angle_rad,
    minimum_valid_distance_m,
):
    """Return the closest finite front reading, or ``None`` when unavailable."""
    if not scan_ranges or angle_increment <= 0.0:
        return None

    valid_ranges = []
    for index, distance in enumerate(scan_ranges):
        angle = angle_min + index * angle_increment
        if (
            abs(angle) <= front_half_angle_rad
            and math.isfinite(distance)
            and distance > minimum_valid_distance_m
        ):
            valid_ranges.append(distance)

    if not valid_ranges:
        return None

    return float(min(valid_ranges))


def target_speed(
    minimum_distance,
    stop_distance_m,
    slow_distance_m,
    slow_speed_mps,
    max_speed_mps,
):
    """Map the nearest obstacle distance to a three-level speed command."""
    if stop_distance_m < 0.0 or slow_distance_m <= stop_distance_m:
        raise ValueError('distance thresholds must be ordered and non-negative')
    if slow_speed_mps < 0.0 or max_speed_mps < slow_speed_mps:
        raise ValueError('speed levels must be ordered and non-negative')

    if minimum_distance is None or minimum_distance < stop_distance_m:
        return 0.0
    if minimum_distance < slow_distance_m:
        return float(slow_speed_mps)
    return float(max_speed_mps)
