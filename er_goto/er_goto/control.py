# SPDX-License-Identifier: GPL-3.0-only
"""Decision logic of the stop-look-move loop, kept free of ROS for unit tests."""

import math
import re
from dataclasses import dataclass

DONE = 'DONE'
TURN = 'TURN'
WALK = 'WALK'
SEARCH = 'SEARCH'

_COMMAND_PREFIX = re.compile(
    r'^\s*(?:please\s+)?'
    r'(?:(?:walk|go|move|head|navigate|come)\s+(?:over\s+)?(?:up\s+to|to|towards|toward)\s+)?'
    r'(?:the\s+)?',
    re.IGNORECASE)


@dataclass(frozen=True)
class Tuning:
    stop_dist: float         # m, stop this far in front of the target
    align_tol: float         # rad, turn first if |bearing| is larger
    turn_speed: float        # rad/s
    walk_speed: float        # m/s
    max_step: float          # m, longest walk per LOOK
    min_step: float          # m, shortest walk, so tiny steps still move the robot
    search_turn_time: float  # s, one SEARCH rotation
    max_motion_time: float   # s, hard cap on any single motion


@dataclass(frozen=True)
class Step:
    action: str
    linear_x: float = 0.0
    angular_z: float = 0.0
    duration: float = 0.0
    reason: str = ''


def decide(bearing, distance, tuning):
    """Next step after a LOOK that found the target.

    bearing: rad, + = left. distance: m, or None when depth is unknown.
    """
    t = tuning
    if abs(bearing) > t.align_tol:
        duration = min(abs(bearing) / t.turn_speed, t.max_motion_time)
        return Step(TURN, angular_z=math.copysign(t.turn_speed, bearing), duration=duration,
                    reason=f'bearing {math.degrees(bearing):+.1f} deg')
    if distance is not None and distance <= t.stop_dist:
        return Step(DONE, reason=f'{distance:.2f} m away, aligned')
    if distance is None:
        step = t.max_step
        reason = 'depth unknown'
    else:
        step = min(max(distance - t.stop_dist, t.min_step), t.max_step)
        reason = f'{distance:.2f} m away'
    duration = min(step / t.walk_speed, t.max_motion_time)
    return Step(WALK, linear_x=t.walk_speed, duration=duration,
                reason=f'{reason}, step {step:.2f} m')


def search_step(tuning):
    """Rotate left in place to look for a target that is not in view."""
    t = tuning
    return Step(SEARCH, angular_z=t.turn_speed,
                duration=min(t.search_turn_time, t.max_motion_time),
                reason='target not in view')


def extract_target(command):
    """'walk to the red chair.' -> 'red chair'; a bare target is returned unchanged."""
    target = _COMMAND_PREFIX.sub('', command, count=1)
    return target.strip().rstrip('.!?').strip()
