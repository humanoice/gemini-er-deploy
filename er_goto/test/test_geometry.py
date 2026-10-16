# SPDX-License-Identifier: GPL-3.0-only
"""Unit tests for the geometry and Gemini parsing helpers (test stage 1). No ROS needed."""

import math

import numpy as np
import pytest

from er_goto import geometry
from er_goto.gemini_client import Point, ResponseParseError, parse_points, point_to_pixel


# ------------------------------------------------------------ [y, x] -> pixel

def test_point_is_y_then_x():
    # y=100 (near the top), x=900 (near the right edge) on a 640x480 image
    assert point_to_pixel(Point(y=100, x=900, label='chair'), 640, 480) == (576, 48)


def test_point_centre_and_corners():
    assert point_to_pixel(Point(500, 500, ''), 640, 480) == (320, 240)
    assert point_to_pixel(Point(0, 0, ''), 640, 480) == (0, 0)
    assert point_to_pixel(Point(1000, 1000, ''), 640, 480) == (639, 479)


# ------------------------------------------------------------------- bearing

def test_bearing_sign():
    fx, cx = 600.0, 320.0
    assert geometry.pixel_to_bearing(320, fx, cx) == 0.0
    # Pixel right of centre -> target on the right -> negative (turn right).
    assert geometry.pixel_to_bearing(500, fx, cx) < 0.0
    assert geometry.pixel_to_bearing(100, fx, cx) > 0.0


def test_bearing_magnitude_and_offset():
    fx, cx = 600.0, 320.0
    assert geometry.pixel_to_bearing(920, fx, cx) == pytest.approx(-math.pi / 4)
    assert geometry.pixel_to_bearing(320, fx, cx, yaw_offset=0.1) == pytest.approx(0.1)


# --------------------------------------------------------------------- depth

def test_depth_median_ignores_zeros():
    depth_mm = np.zeros((480, 640), dtype=np.uint16)
    depth_mm[235:246, 315:326] = 1500
    depth_mm[240, 320] = 0          # holes inside the patch
    depth_mm[238:240, 318:323] = 0
    depth_m = geometry.depth_to_meters(depth_mm)
    assert geometry.depth_at(depth_m, 320, 240, radius=5) == pytest.approx(1.5)


def test_depth_none_when_no_reading():
    depth_m = np.zeros((480, 640), dtype=np.float32)
    assert geometry.depth_at(depth_m, 320, 240) is None


def test_depth_patch_clipped_at_image_edge():
    depth_m = np.full((480, 640), 2.0, dtype=np.float32)
    assert geometry.depth_at(depth_m, 0, 0) == pytest.approx(2.0)
    assert geometry.depth_at(depth_m, 639, 479) == pytest.approx(2.0)


def test_float_depth_nan_is_invalid():
    depth = np.full((4, 4), np.nan, dtype=np.float32)
    depth[1, 1] = 1.2
    depth_m = geometry.depth_to_meters(depth)
    assert geometry.depth_at(depth_m, 1, 1, radius=2) == pytest.approx(1.2)


def test_scale_pixel():
    assert geometry.scale_pixel(320, 240, (640, 480), (640, 480)) == (320, 240)
    assert geometry.scale_pixel(320, 240, (640, 480), (1280, 720)) == (640, 360)
    assert geometry.scale_pixel(639, 479, (640, 480), (320, 240)) == (319, 239)


# ------------------------------------------------------------------ obstacle

def _scene(distance_m=3.0):
    return np.full((480, 640), distance_m, dtype=np.float32)


def test_clear_path():
    blocked, nearest = geometry.check_obstacle(_scene(), obstacle_dist=0.5)
    assert not blocked
    assert nearest == pytest.approx(3.0)


def test_obstacle_in_corridor():
    depth = _scene()
    depth[300:400, 280:360] = 0.4   # centre third, lower part
    blocked, nearest = geometry.check_obstacle(depth, obstacle_dist=0.5)
    assert blocked
    assert nearest == pytest.approx(0.4)


def test_obstacle_outside_corridor_ignored():
    depth = _scene()
    depth[:150, :] = 0.3            # upper third
    depth[:, :200] = 0.3            # left third
    assert not geometry.check_obstacle(depth, obstacle_dist=0.5)[0]


def test_readings_below_min_depth_ignored():
    depth = _scene()
    depth[300:400, 280:360] = 0.05
    assert not geometry.check_obstacle(depth, obstacle_dist=0.5, min_depth=0.1)[0]


def test_no_valid_depth_counts_as_blocked():
    blocked, nearest = geometry.check_obstacle(_scene(0.0), obstacle_dist=0.5)
    assert blocked
    assert nearest is None


# ------------------------------------------------------------ image decoding

def test_rgb8_with_row_padding():
    h, w, step = 2, 3, 12           # 9 bytes of pixels + 3 bytes padding per row
    rows = [bytes([10, 20, 30] * w) + b'\xff' * 3, bytes([40, 50, 60] * w) + b'\xff' * 3]
    image = geometry.image_to_array('rgb8', h, w, step, False, b''.join(rows))
    assert image.shape == (2, 3, 3)
    assert image[0, 0].tolist() == [10, 20, 30]
    bgr = geometry.to_bgr(image, 'rgb8')
    assert bgr[1, 2].tolist() == [60, 50, 40]


def test_16uc1_little_and_big_endian():
    values = np.array([[1000, 2000], [0, 65535]], dtype=np.uint16)
    little = geometry.image_to_array('16UC1', 2, 2, 4, False, values.astype('<u2').tobytes())
    big = geometry.image_to_array('16UC1', 2, 2, 4, True, values.astype('>u2').tobytes())
    assert little.tolist() == values.tolist()
    assert big.tolist() == values.tolist()
    assert geometry.depth_to_meters(little)[0, 1] == pytest.approx(2.0)


def test_unsupported_encoding():
    with pytest.raises(ValueError):
        geometry.image_to_array('yuv422', 1, 1, 2, False, b'\x00\x00')


# ------------------------------------------------------------ Gemini parsing

def test_parse_plain_json():
    points = parse_points('[{"point": [412, 655], "label": "chair"}]')
    assert points == [Point(y=412.0, x=655.0, label='chair')]


def test_parse_fenced_json():
    text = 'Here it is:\n```json\n[\n  {"point": [100, 200], "label": "chair"}\n]\n```\n'
    assert parse_points(text) == [Point(100.0, 200.0, 'chair')]


def test_parse_plain_fence():
    assert parse_points('```\n[{"point": [1, 2]}]\n```') == [Point(1.0, 2.0, '')]


def test_empty_list_means_no_target():
    assert parse_points('[]') == []
    assert parse_points('```json\n[]\n```') == []


def test_single_object_accepted():
    assert parse_points('{"point": [5, 6], "label": "chair"}') == [Point(5.0, 6.0, 'chair')]


@pytest.mark.parametrize('text', [
    '',
    'I cannot see a chair.',
    '```json\n[{"point": [1, 2]\n```',
    '[{"label": "chair"}]',
    '[{"point": [1, 2, 3]}]',
    '[{"point": ["a", "b"]}]',
    '[{"point": [1200, 50]}]',
    '"chair"',
])
def test_parse_rejects_bad_answers(text):
    with pytest.raises(ResponseParseError):
        parse_points(text)
