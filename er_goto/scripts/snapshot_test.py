#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Offline check of Gemini pointing on saved photos (test stage 2). No ROS needed.

    python3 er_goto/scripts/snapshot_test.py --target chair photo1.jpg photo2.jpg ...

Draws every returned point on a copy of each photo, prints the latency per
photo and the median. GEMINI_API_KEY comes from the environment or the
repository's .env file.
"""

import argparse
import os
import statistics
import sys
import time
from pathlib import Path

import cv2

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parent
sys.path.insert(0, str(PACKAGE_ROOT))

from er_goto import control  # noqa: E402
from er_goto.gemini_client import (DEFAULT_MODEL, GeminiError, GeminiPointer,  # noqa: E402
                                   ResponseParseError, point_to_pixel)


def load_dotenv(path):
    """Set KEY=VALUE lines from path in os.environ, without overriding existing values."""
    if not path.is_file():
        return
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#') or '=' not in line:
            continue
        key, value = line.split('=', 1)
        key = key.strip().removeprefix('export ').strip()
        os.environ.setdefault(key, value.strip().strip('"').strip("'"))


def resize_to_width(image, width):
    if width <= 0 or image.shape[1] <= width:
        return image
    height = round(image.shape[0] * width / image.shape[1])
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)


def draw(image, points):
    out = image.copy()
    h, w = out.shape[:2]
    for i, point in enumerate(points):
        u, v = point_to_pixel(point, w, h)
        color = (0, 0, 255) if i == 0 else (0, 200, 255)  # first point is the one er_goto uses
        cv2.circle(out, (u, v), 8, color, 2)
        cv2.drawMarker(out, (u, v), color, cv2.MARKER_CROSS, 24, 1)
        cv2.putText(out, point.label or '?', (u + 10, v - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, color, 1, cv2.LINE_AA)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('images', nargs='+', type=Path, help='photos to test')
    parser.add_argument('--target', required=True, help='object or command, e.g. chair')
    parser.add_argument('--out', type=Path, default=Path('snapshot_out'),
                        help='directory for annotated copies (default: ./snapshot_out)')
    parser.add_argument('--model', default=DEFAULT_MODEL)
    parser.add_argument('--thinking-level', default='low')
    parser.add_argument('--timeout', type=float, default=15.0, help='seconds per request')
    parser.add_argument('--width', type=int, default=640,
                        help='resize photos to this width like the robot camera (0 = keep)')
    args = parser.parse_args()

    load_dotenv(REPO_ROOT / '.env')
    target = control.extract_target(args.target)
    pointer = GeminiPointer(model=args.model, thinking_level=args.thinking_level,
                            timeout=args.timeout, retries=0)
    args.out.mkdir(parents=True, exist_ok=True)

    latencies = []
    found = 0
    for path in args.images:
        image = cv2.imread(str(path))
        if image is None:
            print(f'{path}: cannot read image')
            continue
        image = resize_to_width(image, args.width)
        ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            print(f'{path}: JPEG encoding failed')
            continue

        started = time.monotonic()
        try:
            points, text = pointer.point(jpeg.tobytes(), target)
        except (GeminiError, ResponseParseError) as exc:
            latency = time.monotonic() - started
            print(f'{path}: {type(exc).__name__}: {exc} ({latency:.1f} s)')
            continue
        latency = time.monotonic() - started
        latencies.append(latency)

        if points:
            found += 1
            h, w = image.shape[:2]
            u, v = point_to_pixel(points[0], w, h)
            summary = (f'{len(points)} point(s); first "{points[0].label}" at '
                       f'[y={points[0].y:.0f}, x={points[0].x:.0f}] -> pixel ({u}, {v})')
        else:
            summary = 'not visible'
        out_path = args.out / f'{path.stem}_er.jpg'
        cv2.imwrite(str(out_path), draw(image, points))
        print(f'{path}: {summary}, {latency:.1f} s -> {out_path}')

    print(f'\nTarget "{target}" found in {found}/{len(args.images)} photo(s)')
    if latencies:
        print(f'Gemini latency: median {statistics.median(latencies):.1f} s, '
              f'min {min(latencies):.1f} s, max {max(latencies):.1f} s')
    return 0 if latencies else 1


if __name__ == '__main__':
    sys.exit(main())
