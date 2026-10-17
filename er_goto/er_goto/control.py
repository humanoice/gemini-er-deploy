# SPDX-License-Identifier: GPL-3.0-only
"""Decision logic of the look-while-walking loop, kept free of ROS for unit tests.

A Gemini answer takes seconds, so the robot keeps walking on the newest one.
Each answer is tied to the dead-reckoned pose at the moment its photo was taken,
which turns "the chair was 20 deg left of where I stood then" into a bearing and
distance from where the robot is now.
"""

import math
import re
from dataclasses import dataclass

DONE = 'DONE'
TURN = 'TURN'
WALK = 'WALK'
STAND = 'STAND'
PURSUE = 'PURSUE'
SEARCH = 'SEARCH'

# Placeholder range for a target seen without valid depth: far enough that
# walking barely changes its bearing.
UNRANGED_DISTANCE = 5.0

_COMMAND_PREFIX = re.compile(
    r'^\s*(?:please\s+)?'
    r'(?:(?:walk|go|move|head|navigate|come)\s+(?:over\s+)?(?:up\s+to|to|towards|toward)\s+)?'
    r'(?:the\s+)?',
    re.IGNORECASE)


class GaveUp(Exception):
    """The target cannot be reached: not found, path blocked, or a run limit hit."""


@dataclass(frozen=True)
class Tuning:
    stop_dist: float            # m, walk until this far in front of the target
    arrive_tol: float           # m, a standing LOOK within stop_dist + this counts as arrived
    align_tol: float            # rad, arrived only with the target within this of straight ahead
    turn_in_place_angle: float  # rad, turn in place if |bearing| is larger, else walk and turn
    k_turn: float               # 1/s, angular_z = k_turn * bearing
    turn_speed: float           # rad/s, fastest turn, also the SEARCH rotation
    walk_speed: float           # m/s, fastest walk
    min_walk_speed: float       # m/s, slowest walk, so the gait still steps forward
    slow_dist: float            # m, slow down over this last stretch before stop_dist
    settle_time: float          # s, stand still before a standing LOOK
    search_turn_time: float     # s, one SEARCH rotation
    max_search: int             # standing LOOKs in a row without the target before giving up
    max_blind_time: float       # s, stop if the newest fix's photo is older
    max_blind_dist: float       # m, stop after walking this far since the newest fix's photo


@dataclass(frozen=True)
class Step:
    action: str
    linear_x: float = 0.0
    angular_z: float = 0.0
    reason: str = ''


@dataclass(frozen=True)
class Pose:
    """Dead-reckoned pose in the frame where the run started: m, m, rad (+ = left)."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def advance(pose, linear_x, angular_z, dt):
    """Pose after driving linear_x / angular_z (body frame) for dt seconds."""
    heading = pose.yaw + 0.5 * angular_z * dt  # midpoint of the arc
    return Pose(pose.x + linear_x * dt * math.cos(heading),
                pose.y + linear_x * dt * math.sin(heading),
                _wrap(pose.yaw + angular_z * dt))


def place(pose, bearing, distance):
    """Odometry-frame point seen at bearing / distance from pose."""
    heading = pose.yaw + bearing
    return pose.x + distance * math.cos(heading), pose.y + distance * math.sin(heading)


def locate(pose, point):
    """(bearing, distance) of an odometry-frame point as seen from pose."""
    dx, dy = point[0] - pose.x, point[1] - pose.y
    return _wrap(math.atan2(dy, dx) - pose.yaw), math.hypot(dx, dy)


def pursue(bearing, distance, tuning):
    """Velocity toward a target at bearing (rad, + = left) and distance (m, None = unknown).

    Walks and turns at once, slower when misaligned or within slow_dist of
    stop_dist, and turns in place beyond turn_in_place_angle. DONE once at
    stop_dist and within align_tol / 2: tighter than arrived(), so the standing
    LOOK that confirms it usually passes.
    """
    t = tuning
    angular_z = min(max(t.k_turn * bearing, -t.turn_speed), t.turn_speed)
    bearing_text = f'bearing {math.degrees(bearing):+.1f} deg'
    if distance is not None and distance <= t.stop_dist:
        if abs(bearing) <= t.align_tol / 2:
            return Step(DONE, reason=f'{distance:.2f} m away, aligned')
        return Step(TURN, angular_z=angular_z, reason=f'{distance:.2f} m away, {bearing_text}')
    if abs(bearing) > t.turn_in_place_angle:
        return Step(TURN, angular_z=angular_z, reason=bearing_text)
    heading = 1.0 - (bearing / t.turn_in_place_angle) ** 2
    approach = 1.0 if distance is None else min((distance - t.stop_dist) / t.slow_dist, 1.0)
    linear_x = max(t.walk_speed * heading * approach, t.min_walk_speed)
    dist_text = 'depth unknown' if distance is None else f'{distance:.2f} m away'
    return Step(WALK, linear_x=linear_x, angular_z=angular_z,
                reason=f'{dist_text}, {bearing_text}')


def arrived(bearing, distance, tuning):
    """Whether a standing LOOK shows the robot in front of the target."""
    t = tuning
    return (distance is not None and distance <= t.stop_dist + t.arrive_tol
            and abs(bearing) <= t.align_tol)


@dataclass(frozen=True)
class Look:
    """A LOOK request, tagged with the dead-reckoned state when it was made."""
    standing: bool
    time: float  # time.monotonic(); a standing LOOK needs a frame received after this
    pose: Pose
    path: float  # m walked since the start


@dataclass(frozen=True)
class Command:
    linear_x: float = 0.0
    angular_z: float = 0.0
    look: Look = None  # start this LOOK now
    done: bool = False


@dataclass(frozen=True)
class _Fix:
    point: tuple   # odometry frame
    ranged: bool   # False: depth was unknown, point is UNRANGED_DISTANCE along the bearing
    time: float    # when its photo was taken
    path: float


class Navigator:
    """Walks toward the newest Gemini fix and decides when to stop and look.

    The node calls tick() at its publish rate and publishes the returned command.
    When the command carries a Look, the node photographs, asks Gemini and
    reports back with seen(), missed() or failed(). At most one LOOK is in flight.

    STAND   zero; after settle_time, a standing LOOK, whose answer decides:
            arrived -> DONE, seen -> PURSUE, not seen -> SEARCH. A moving LOOK
            still in flight resumes PURSUE when it shows the target ahead, but
            only finishes the run if its photo was taken within arrive_tol /
            (align_tol / 2) of where the robot stopped. After a blocked path,
            only a standing LOOK decides.
    PURSUE  walk / turn toward the fix, with moving LOOKs back to back. Back to
            STAND when dead reckoning says arrived, the fix is older than
            max_blind_time or max_blind_dist, the path is blocked, or stop().
    SEARCH  rotate left for search_turn_time, then STAND.
    """

    def __init__(self, tuning, now, log=None):
        self.t = tuning
        self.mode = STAND
        self.pose = Pose()
        self.path = 0.0
        self._log = log or (lambda message: None)
        self._since = now
        self._last_tick = now
        self._command = (0.0, 0.0)
        self._fix = None
        self._looking = False
        self._latency = 0.0    # s, from request to answer of the latest LOOK
        self._misses = 0
        self._blocked = False  # last stop was for a blocked path
        self._report = False   # log the next PURSUE command
        self._search_until = 0.0

    def target(self):
        """(bearing, distance) of the newest fix from the current pose, or None.

        distance is None when the fix had no valid depth.
        """
        if self._fix is None:
            return None
        bearing, distance = locate(self.pose, self._fix.point)
        return bearing, (distance if self._fix.ranged else None)

    def tick(self, now, walk_blocked):
        """Advance dead reckoning to now and return the Command to publish.

        walk_blocked() returns why the robot must not walk forward, or None; it is
        called only when the command would walk. Raises GaveUp.
        """
        linear_x, angular_z = self._command
        dt = max(now - self._last_tick, 0.0)
        self.pose = advance(self.pose, linear_x, angular_z, dt)
        self.path += abs(linear_x) * dt
        self._last_tick = now
        command = self._decide(now, walk_blocked)
        self._command = (command.linear_x, command.angular_z)
        return command

    def _decide(self, now, walk_blocked):
        t = self.t
        if self.mode == DONE:
            return Command(done=True)
        if self.mode == SEARCH:
            if now < self._search_until:
                return Command(angular_z=t.turn_speed)
            self._stand(now, 'search turn done')
        if self.mode == STAND:
            if self._looking or now - self._since < t.settle_time:
                return Command()
            return Command(look=self._look(now, standing=True))

        fix = self._fix
        if now - fix.time > t.max_blind_time:
            return self._stand(now, f'newest fix is {now - fix.time:.1f} s old')
        if self.path - fix.path > t.max_blind_dist:
            return self._stand(now, f'walked {self.path - fix.path:.2f} m since the newest fix')
        bearing, distance = self.target()
        step = pursue(bearing, distance, t)
        if step.action == DONE:
            return self._stand(now, f'reached by dead reckoning ({step.reason}), confirming')
        if step.linear_x > 0.0:
            blocked = walk_blocked()
            if blocked:
                if self._blocked:
                    raise GaveUp(f'cannot walk: {blocked}')
                self._blocked = True
                return self._stand(now, f'path blocked: {blocked}')
            self._blocked = False
        if self._report:
            self._report = False
            self._log(f'PURSUE {step.action}: linear.x={step.linear_x:+.2f} '
                      f'angular.z={step.angular_z:+.2f} ({step.reason})')
        look = None
        if not (self._looking or self._arrives_before_answer(distance, step.linear_x)):
            look = self._look(now, standing=False)
        return Command(step.linear_x, step.angular_z, look=look)

    def _arrives_before_answer(self, distance, linear_x):
        """Whether dead reckoning reaches stop_dist before a LOOK started now would answer.

        Such a LOOK is wasted: the robot would stand waiting for it before the
        standing LOOK that confirms arrival. Walking never drops below
        min_walk_speed, so this errs towards looking.
        """
        if distance is None or linear_x <= 0.0:
            return False
        return distance - self.t.stop_dist < self.t.min_walk_speed * self._latency

    def _look(self, now, standing):
        self._looking = True
        return Look(standing=standing, time=now, pose=self.pose, path=self.path)

    def _stand(self, now, reason):
        if self.mode != STAND:
            self._log(f'STAND: {reason}')
        self.mode = STAND
        self._since = now
        return Command()

    def stop(self, now, reason):
        """Stand still and look again, e.g. after the node's watchdog fired."""
        self._stand(now, reason)

    def seen(self, look, bearing, distance, now):
        """A LOOK found the target at bearing / distance (m, None = unknown) from look.pose."""
        self._looking = False
        self._latency = now - look.time
        self._misses = 0
        ranged = distance is not None
        point = place(look.pose, bearing, distance if ranged else UNRANGED_DISTANCE)
        self._fix = _Fix(point, ranged, look.time, look.path)
        if self.mode == PURSUE:
            self._report = True
            return
        if self.mode != STAND:
            return
        bearing, distance = self.target()
        confirmed = look.standing or self._near(look)
        if not confirmed and (self._blocked or arrived(bearing, distance, self.t)):
            return  # photo taken while still walking; the standing LOOK decides
        if confirmed and arrived(bearing, distance, self.t):
            self.mode = DONE
            self._log(f'DONE: {distance:.2f} m away, bearing {math.degrees(bearing):+.1f} deg')
            return
        self.mode = PURSUE
        self._report = True

    def _near(self, look):
        """Whether look's photo was taken about where the robot stands now."""
        return (self.path - look.path <= self.t.arrive_tol
                and abs(_wrap(self.pose.yaw - look.pose.yaw)) <= self.t.align_tol / 2)

    def missed(self, look, now):
        """A LOOK did not find the target, or its answer was unusable. Raises GaveUp."""
        self._looking = False
        self._latency = now - look.time
        if not (self.mode == STAND and look.standing):
            return  # keep going on the newest fix until max_blind_time / max_blind_dist
        self._misses += 1
        if self._misses >= self.t.max_search:
            raise GaveUp(f'target not found after {self._misses} standing looks')
        self._fix = None
        self._blocked = False
        self.mode = SEARCH
        self._search_until = now + self.t.search_turn_time
        self._log(f'SEARCH: target not in view ({self._misses}/{self.t.max_search})')

    def failed(self, look):
        """A LOOK got no answer (Gemini error); the next tick asks again."""
        self._looking = False


def extract_target(command):
    """'walk to the red chair.' -> 'red chair'; a bare target is returned unchanged."""
    target = _COMMAND_PREFIX.sub('', command, count=1)
    return target.strip().rstrip('.!?').strip()
