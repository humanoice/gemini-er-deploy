# SPDX-License-Identifier: GPL-3.0-only
"""Unit tests for the look-while-walking decisions. No ROS needed."""

import dataclasses
import math

import pytest

from er_goto import control

TUNING = control.Tuning(stop_dist=0.8, arrive_tol=0.15, align_tol=0.15, turn_in_place_angle=0.5,
                        k_turn=1.5, turn_speed=0.5, walk_speed=0.35, min_walk_speed=0.15,
                        slow_dist=1.0, settle_time=0.3, search_turn_time=1.5, max_search=3,
                        max_blind_time=10.0, max_blind_dist=2.5, min_progress=0.3, max_stalls=2)

DT = 0.05


def clear():
    return None


# ------------------------------------------------------------- dead reckoning

def test_advance_straight():
    pose = control.advance(control.Pose(), linear_x=0.5, angular_z=0.0, dt=2.0)
    assert (pose.x, pose.y, pose.yaw) == pytest.approx((1.0, 0.0, 0.0))


def test_advance_turn_in_place():
    pose = control.advance(control.Pose(), linear_x=0.0, angular_z=0.5, dt=2.0)
    assert (pose.x, pose.y, pose.yaw) == pytest.approx((0.0, 0.0, 1.0))


def test_place_locate_round_trip():
    pose = control.Pose(1.0, 2.0, 0.3)
    point = control.place(pose, bearing=0.2, distance=3.0)
    assert control.locate(pose, point) == pytest.approx((0.2, 3.0))


def test_locate_after_moving():
    point = control.place(control.Pose(), bearing=0.0, distance=3.0)
    assert control.locate(control.Pose(1.0, 0.0, 0.0), point) == pytest.approx((0.0, 2.0))
    assert control.locate(control.Pose(0.0, 0.0, 0.3), point) == pytest.approx((-0.3, 3.0))


# --------------------------------------------------------------------- pursue

def test_pursue_walks_straight_at_full_speed():
    step = control.pursue(bearing=0.0, distance=3.0, tuning=TUNING)
    assert step.action == control.WALK
    assert step.linear_x == pytest.approx(0.35)
    assert step.angular_z == 0.0


def test_pursue_turns_while_walking():
    step = control.pursue(bearing=0.2, distance=3.0, tuning=TUNING)
    assert step.action == control.WALK
    assert step.angular_z == pytest.approx(0.3)
    assert step.linear_x == pytest.approx(0.35 * (1 - (0.2 / 0.5) ** 2))


def test_pursue_turns_in_place_when_far_off():
    step = control.pursue(bearing=-0.7, distance=3.0, tuning=TUNING)
    assert step.action == control.TURN
    assert step.linear_x == 0.0
    assert step.angular_z == pytest.approx(-0.5)


def test_pursue_slows_near_target():
    assert control.pursue(0.0, 1.3, TUNING).linear_x == pytest.approx(0.35 * 0.5)
    assert control.pursue(0.0, 0.85, TUNING).linear_x == pytest.approx(0.15)


def test_pursue_full_speed_when_depth_unknown():
    step = control.pursue(bearing=0.0, distance=None, tuning=TUNING)
    assert step.action == control.WALK
    assert step.linear_x == pytest.approx(0.35)


def test_pursue_done_only_when_aligned_tightly():
    assert control.pursue(0.05, 0.8, TUNING).action == control.DONE
    step = control.pursue(0.1, 0.7, TUNING)
    assert step.action == control.TURN
    assert step.linear_x == 0.0
    assert step.angular_z == pytest.approx(0.15)


def test_arrived():
    assert control.arrived(0.1, 0.9, TUNING)
    assert not control.arrived(0.2, 0.9, TUNING)
    assert not control.arrived(0.0, 1.0, TUNING)
    assert not control.arrived(0.0, None, TUNING)


# ------------------------------------------------------------------ Navigator

def run_ticks(nav, start, end, blocked=clear):
    """Tick every DT from start until end; returns (time after, last command, looks issued)."""
    now, command, looks = start, None, []
    while now < end - 1e-9:
        command = nav.tick(now, blocked)
        if command.look is not None:
            looks.append(command.look)
        now += DT
    return now, command, looks


def standing_look(nav, start=0.0):
    """Tick until the navigator asks for a standing LOOK; returns (time, look)."""
    now = start
    while True:
        command = nav.tick(now, clear)
        assert command.linear_x == 0.0 and command.angular_z == 0.0
        if command.look is not None:
            assert command.look.standing
            return now, command.look
        now += DT
        assert now < start + 5.0


def walk_until_standing_look(nav, start):
    """Tick a PURSUE until dead reckoning stops it for a standing LOOK; returns (time, look)."""
    now = start
    while nav.mode == control.PURSUE:
        assert nav.tick(now, clear).look is None  # no moving LOOK in flight
        now += DT
        assert now < start + 30.0
    return standing_look(nav, now)


def pursuing(bearing, distance, tuning=TUNING, latency=3.0):
    """A navigator that saw the target from where it started; returns (nav, time)."""
    nav = control.Navigator(tuning, 0.0)
    now, look = standing_look(nav)
    now += latency
    nav.seen(look, bearing, distance, now)
    assert nav.mode == control.PURSUE
    return nav, now


def test_stands_and_settles_before_first_look():
    nav = control.Navigator(TUNING, 0.0)
    assert nav.tick(0.1, clear) == control.Command()
    now, look = standing_look(nav)
    assert now >= TUNING.settle_time
    assert look.pose == control.Pose()
    assert nav.tick(now + DT, clear).look is None  # one LOOK at a time


def test_walks_and_looks_while_walking():
    nav, now = pursuing(0.0, 3.0)
    command = nav.tick(now, clear)
    assert command.linear_x == pytest.approx(0.35)
    assert command.look is not None and not command.look.standing
    _, command, looks = run_ticks(nav, now + DT, now + 2.0)
    assert command.linear_x == pytest.approx(0.35)
    assert looks == []  # previous LOOK still in flight


def test_moving_fix_is_placed_from_where_its_photo_was_taken():
    nav, now = pursuing(0.0, 4.0)
    look = nav.tick(now, clear).look
    now, _, _ = run_ticks(nav, now + DT, now + 2.0 + DT)
    walked = nav.pose.x - look.pose.x
    assert walked == pytest.approx(0.7, abs=0.02)
    nav.seen(look, bearing=0.0, distance=4.0, now=now)
    bearing, distance = nav.target()
    assert bearing == pytest.approx(0.0)
    assert distance == pytest.approx(4.0 - walked)


def test_turn_since_photo_corrects_bearing():
    nav, now = pursuing(-0.8, 3.0)  # far off to the right: turn in place
    look = nav.tick(now, clear).look
    now, command, _ = run_ticks(nav, now + DT, now + 0.5 + DT)
    assert command.linear_x == 0.0
    turned = nav.pose.yaw - look.pose.yaw
    assert turned == pytest.approx(-0.25, abs=0.01)
    nav.seen(look, bearing=-0.8, distance=3.0, now=now)
    assert nav.target()[0] == pytest.approx(-0.8 - turned)


def test_arrival_is_confirmed_by_a_standing_look():
    nav, now = pursuing(0.0, 2.0)
    moving = nav.tick(now, clear).look
    now, command, _ = run_ticks(nav, now + DT, now + 6.0)
    assert nav.mode == control.STAND  # dead reckoning reached stop_dist
    assert command == control.Command()  # waits for the moving LOOK in flight
    nav.failed(moving)
    now, look = standing_look(nav, now)
    nav.seen(look, bearing=0.05, distance=0.9, now=now + 3.0)
    assert nav.tick(now + 3.0, clear).done


def test_standing_look_short_of_target_resumes():
    nav, now = pursuing(0.0, 1.0)
    now, _, looks = run_ticks(nav, now, now + 3.0)
    assert nav.mode == control.STAND
    assert [look.standing for look in looks] == [True]  # no moving LOOK: it would answer late
    look = looks[0]
    nav.seen(look, bearing=0.0, distance=1.2, now=now)
    assert nav.mode == control.PURSUE
    assert nav.tick(now, clear).linear_x > 0.0


def test_moving_fix_resumes_from_stand_when_target_still_ahead():
    nav, now = pursuing(0.0, 3.0)
    moving = nav.tick(now, clear).look
    now, _, _ = run_ticks(nav, now + DT, now + 1.0)  # 0.35 m past the photo
    nav.stop(now, 'newest fix is too old')
    nav.seen(moving, bearing=0.0, distance=2.5, now=now)
    assert nav.mode == control.PURSUE
    assert nav.tick(now, clear).linear_x > 0.0


def test_moving_fix_from_further_back_cannot_finish():
    nav, now = pursuing(0.0, 3.0)
    moving = nav.tick(now, clear).look
    now, _, _ = run_ticks(nav, now + DT, now + 1.0)  # 0.35 m past the photo
    nav.stop(now, 'test')
    nav.seen(moving, bearing=0.0, distance=1.2, now=now)  # 0.85 m from here: arrived?
    assert nav.mode == control.STAND
    _, look = standing_look(nav, now)
    assert look.standing


def test_moving_fix_does_not_resume_after_blocked_path():
    nav, now = pursuing(0.0, 3.0)
    moving = nav.tick(now, clear).look
    now, _, _ = run_ticks(nav, now + DT, now + 1.0)
    nav.tick(now, lambda: 'obstacle in path (0.40 m)')
    nav.seen(moving, bearing=0.0, distance=2.5, now=now)
    assert nav.mode == control.STAND
    _, look = standing_look(nav, now)
    assert look.standing


def test_skips_moving_look_that_would_answer_after_arrival():
    nav, now = pursuing(0.0, 1.2)
    assert nav.tick(now, clear).look is None  # 0.4 m to go < 0.15 m/s x 3 s
    nav, now = pursuing(0.0, 1.2, latency=1.0)
    assert nav.tick(now, clear).look is not None


def test_moving_fix_from_where_it_stopped_confirms_arrival():
    nav, now = pursuing(0.0, 1.0, latency=0.2)  # walks at min_walk_speed
    first = nav.tick(now, clear).look
    now, _, _ = run_ticks(nav, now + DT, now + 1.0)
    nav.failed(first)
    last = nav.tick(now, clear).look  # photo about 0.05 m before dead reckoning arrives
    now, _, _ = run_ticks(nav, now + DT, now + 1.0)
    assert nav.mode == control.STAND
    assert nav.path - last.path < TUNING.arrive_tol
    nav.seen(last, bearing=0.0, distance=0.95, now=now)
    assert nav.target()[1] == pytest.approx(0.95 - (nav.path - last.path))
    assert nav.tick(now, clear).done


def test_moving_miss_keeps_walking():
    nav, now = pursuing(0.0, 3.0)
    nav.missed(nav.tick(now, clear).look, now)
    command = nav.tick(now + DT, clear)
    assert nav.mode == control.PURSUE
    assert command.linear_x > 0.0
    assert command.look is not None


def test_stops_when_fix_too_old():
    slow = dataclasses.replace(TUNING, walk_speed=0.15)
    nav, now = pursuing(0.0, 3.0, slow)
    nav.tick(now, clear)
    run_ticks(nav, now + DT, 0.3 + TUNING.max_blind_time - DT)  # fix photo taken at 0.3 s
    assert nav.mode == control.PURSUE
    run_ticks(nav, 0.3 + TUNING.max_blind_time - DT, 0.3 + TUNING.max_blind_time + 2 * DT)
    assert nav.mode == control.STAND


def test_stops_after_walking_too_far_on_one_fix():
    nav, now = pursuing(0.0, 10.0, dataclasses.replace(TUNING, max_blind_time=60.0))
    nav.tick(now, clear)
    run_ticks(nav, now + DT, now + 2.5 / 0.35 + 0.2)
    assert nav.mode == control.STAND
    assert nav.path == pytest.approx(2.5, abs=0.05)


def test_blocked_path_stops_then_gives_up():
    nav, now = pursuing(0.0, 3.0)
    nav.failed(nav.tick(now, clear).look)
    command = nav.tick(now + DT, lambda: 'obstacle in path (0.40 m)')
    assert nav.mode == control.STAND
    assert command == control.Command()
    now, look = standing_look(nav, now + 2 * DT)
    nav.seen(look, bearing=0.0, distance=2.5, now=now)
    with pytest.raises(control.GaveUp, match='cannot walk'):
        nav.tick(now, lambda: 'obstacle in path (0.40 m)')


def test_blocked_but_arrived_is_done():
    nav, now = pursuing(0.0, 3.0)
    nav.failed(nav.tick(now, clear).look)
    nav.tick(now + DT, lambda: 'obstacle in path (0.45 m)')
    now, look = standing_look(nav, now + 2 * DT)
    nav.seen(look, bearing=0.0, distance=0.45, now=now)
    assert nav.tick(now, clear).done


def test_search_then_give_up():
    nav = control.Navigator(TUNING, 0.0)
    now, look = standing_look(nav)
    nav.missed(look, now)
    assert nav.mode == control.SEARCH
    _, command, _ = run_ticks(nav, now, now + 1.0)
    assert command.angular_z == pytest.approx(TUNING.turn_speed)
    assert command.linear_x == 0.0
    now, _, _ = run_ticks(nav, now + 1.0, now + TUNING.search_turn_time + DT)
    assert nav.mode == control.STAND
    assert nav.pose.yaw == pytest.approx(TUNING.turn_speed * TUNING.search_turn_time, abs=0.03)
    for _ in range(TUNING.max_search - 2):
        now, look = standing_look(nav, now)
        nav.missed(look, now)
        now, _, _ = run_ticks(nav, now, now + TUNING.search_turn_time + DT)
    now, look = standing_look(nav, now)
    with pytest.raises(control.GaveUp, match='not found'):
        nav.missed(look, now)


def test_gives_up_when_the_robot_does_not_walk():
    nav, now = pursuing(0.0, 1.12)  # the logged run: the gait stepped in place
    now, look = walk_until_standing_look(nav, now)
    assert nav.path == pytest.approx(0.32, abs=0.01)
    nav.seen(look, bearing=0.0, distance=1.08, now=now + 3.0)
    assert nav.mode == control.PURSUE
    now, look = walk_until_standing_look(nav, now + 3.0)
    with pytest.raises(control.GaveUp, match='not walking'):
        nav.seen(look, bearing=0.0, distance=1.12, now=now + 3.0)


def test_short_walks_add_up_before_judging_progress():
    nav, now = pursuing(0.0, 0.97)  # 0.17 m per walk, within depth noise
    for _ in range(3):  # stall only on the second, after 0.34 m
        now, look = walk_until_standing_look(nav, now)
        nav.seen(look, bearing=0.0, distance=0.97, now=now + 3.0)
        now += 3.0
    now, look = walk_until_standing_look(nav, now)
    with pytest.raises(control.GaveUp, match='not walking'):
        nav.seen(look, bearing=0.0, distance=0.97, now=now + 3.0)


def test_progress_resets_stalls():
    nav, now = pursuing(0.0, 1.12)
    for distance in (1.08, 0.97, 0.97, 0.96):  # stall, 0.11 m of 0.28 m, short walk, stall
        now, look = walk_until_standing_look(nav, now)
        nav.seen(look, bearing=0.0, distance=distance, now=now + 3.0)
        now += 3.0
    assert nav.mode == control.PURSUE


def test_arrival_beats_a_stall():
    nav, now = pursuing(0.0, 1.12, dataclasses.replace(TUNING, min_progress=1.0, max_stalls=1))
    now, look = walk_until_standing_look(nav, now)
    nav.seen(look, bearing=0.0, distance=0.95, now=now + 3.0)  # 0.17 m of 0.32 m, but arrived
    assert nav.tick(now + 3.0, clear).done


def test_zero_max_stalls_turns_the_progress_check_off():
    nav, now = pursuing(0.0, 1.12, dataclasses.replace(TUNING, max_stalls=0))
    for _ in range(4):
        now, look = walk_until_standing_look(nav, now)
        nav.seen(look, bearing=0.0, distance=1.12, now=now + 3.0)
        now += 3.0
    assert nav.mode == control.PURSUE


def test_unranged_fix_walks_on_bearing():
    nav, now = pursuing(0.1, None)
    command = nav.tick(now, clear)
    bearing, distance = nav.target()
    assert distance is None
    assert bearing == pytest.approx(0.1)
    assert command.linear_x == pytest.approx(0.35 * (1 - (0.1 / 0.5) ** 2))
    assert command.angular_z == pytest.approx(0.15)


def test_watchdog_stop_stands():
    nav, now = pursuing(0.0, 3.0)
    nav.tick(now, clear)
    nav.stop(now + DT, 'watchdog fired')
    assert nav.mode == control.STAND
    assert nav.tick(now + 2 * DT, clear) == control.Command()


@pytest.mark.parametrize('command, target', [
    ('chair', 'chair'),
    ('walk to the chair', 'chair'),
    ('Walk to the chair.', 'chair'),
    ('please go towards the red chair near the window', 'red chair near the window'),
    ('move over to the sofa!', 'sofa'),
    ('the chair', 'chair'),
    ('theater seat', 'theater seat'),
    ('walk to', 'walk to'),
])
def test_extract_target(command, target):
    assert control.extract_target(command) == target


@pytest.mark.parametrize('line, parsed', [
    ('go to the black chair', ('go', 'black chair')),
    ('walk to the lady', ('go', 'lady')),
    ('chair', ('go', 'chair')),
    ('  Stop. ', ('stop', None)),
    ('halt', ('stop', None)),
    ('s', ('stop', None)),
    ('QUIT', ('quit', None)),
    ('exit', ('quit', None)),
    ('q', ('quit', None)),
    ('help', ('help', None)),
    ('?', ('help', None)),
    ('', (None, None)),
    ('   ', (None, None)),
    ('.', (None, None)),
    ('stop sign', ('go', 'stop sign')),
])
def test_parse_command(line, parsed):
    assert control.parse_command(line) == parsed


def test_bearing_wraps():
    point = control.place(control.Pose(0.0, 0.0, 3.0), bearing=0.3, distance=1.0)
    bearing, _ = control.locate(control.Pose(0.0, 0.0, 3.0), point)
    assert bearing == pytest.approx(0.3)
    assert abs(control.locate(control.Pose(), point)[0]) <= math.pi
