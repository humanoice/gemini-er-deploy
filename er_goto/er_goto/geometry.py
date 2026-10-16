# SPDX-License-Identifier: GPL-3.0-only
"""Pure image helpers: ROS image decoding, pixel to bearing/distance, obstacle check.

Nothing here imports ROS, so the module can be unit-tested on a laptop.
"""

import math

import numpy as np

# encoding -> (numpy dtype, channels)
_ENCODINGS = {
    'rgb8': (np.uint8, 3),
    'bgr8': (np.uint8, 3),
    'mono8': (np.uint8, 1),
    '8uc1': (np.uint8, 1),
    '16uc1': (np.uint16, 1),
    'mono16': (np.uint16, 1),
    '32fc1': (np.float32, 1),
}


def image_to_array(encoding, height, width, step, is_bigendian, data):
    """Decode the fields of a sensor_msgs/Image into an (h, w) or (h, w, c) array.

    Handles row padding (step > width * pixel size) and big-endian data, so no
    cv_bridge is needed.
    """
    key = encoding.lower()
    if key not in _ENCODINGS:
        raise ValueError(f'unsupported image encoding: {encoding}')
    dtype, channels = _ENCODINGS[key]
    dtype = np.dtype(dtype).newbyteorder('>' if is_bigendian else '<')
    row_bytes = width * channels * dtype.itemsize
    raw = np.frombuffer(data, dtype=np.uint8)
    if step < row_bytes or raw.size < height * step:
        raise ValueError(f'image buffer too small: {raw.size} bytes for {height}x{step}')
    rows = np.ascontiguousarray(raw[:height * step].reshape(height, step)[:, :row_bytes])
    image = rows.view(dtype).reshape(height, width, channels).astype(dtype.newbyteorder('='))
    return image[:, :, 0] if channels == 1 else image


def to_bgr(image, encoding):
    """Convert a decoded colour image to BGR (OpenCV order)."""
    key = encoding.lower()
    if key == 'bgr8':
        return image
    if key == 'rgb8':
        return np.ascontiguousarray(image[:, :, ::-1])
    if key in ('mono8', '8uc1'):
        return np.ascontiguousarray(np.repeat(image[:, :, None], 3, axis=2))
    raise ValueError(f'not a colour encoding: {encoding}')


def depth_to_meters(depth, depth_scale=0.001):
    """Depth image to float32 metres; invalid readings (0, NaN, inf) become 0.

    uint16 images are in units of depth_scale (RealSense default: millimetres);
    float images are already in metres.
    """
    if depth.dtype == np.uint16:
        return depth.astype(np.float32) * np.float32(depth_scale)
    meters = depth.astype(np.float32)
    meters[~np.isfinite(meters)] = 0.0
    return meters


def scale_pixel(u, v, src_size, dst_size):
    """Map pixel (u, v) from an image of src_size (w, h) to one of dst_size (w, h)."""
    (src_w, src_h), (dst_w, dst_h) = src_size, dst_size
    if (src_w, src_h) == (dst_w, dst_h):
        return u, v
    du = min(int(u * dst_w / src_w), dst_w - 1)
    dv = min(int(v * dst_h / src_h), dst_h - 1)
    return du, dv


def pixel_to_bearing(u, fx, cx, yaw_offset=0.0):
    """Horizontal angle (rad) from the robot's heading to pixel column u.

    Positive = target is to the left, matching ROS (+angular.z turns left).
    yaw_offset is the camera's mounting yaw relative to the robot, added as is.
    """
    return -math.atan((u - cx) / fx) + yaw_offset


def depth_at(depth_m, u, v, radius=5):
    """Median depth (m) in a square patch around (u, v), ignoring invalid zeros.

    Returns None when the patch has no valid reading.
    """
    h, w = depth_m.shape[:2]
    patch = depth_m[max(v - radius, 0):min(v + radius + 1, h),
                    max(u - radius, 0):min(u + radius + 1, w)]
    valid = patch[patch > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def check_obstacle(depth_m, obstacle_dist, min_depth=0.1, min_valid_fraction=0.1,
                   percentile=5.0):
    """Look for something closer than obstacle_dist in the walking corridor.

    The corridor is the centre third of the image horizontally and its lower
    two thirds vertically. Readings below min_depth are ignored. If fewer than
    min_valid_fraction of the corridor pixels are valid (camera covered, object
    inside the sensor's minimum range), the path counts as blocked.

    Returns (blocked, nearest_m); nearest_m is None when nothing valid was seen.
    """
    h, w = depth_m.shape[:2]
    roi = depth_m[h // 3:, w // 3:2 * w // 3]
    valid = roi[roi >= min_depth]
    if roi.size == 0 or valid.size < min_valid_fraction * roi.size:
        return True, None
    nearest = float(np.percentile(valid, percentile))
    return nearest < obstacle_dist, nearest
