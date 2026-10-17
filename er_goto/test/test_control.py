# SPDX-License-Identifier: GPL-3.0-only
"""Unit tests for the stop-look-move decisions. No ROS needed."""

import dataclasses

import pytest

from er_goto import control

TUNING = control.Tuning(stop_dist=0.8, align_tol=0.15, turn_in_place_angle=0.5,
                        turn_speed=0.4, walk_speed=0.25, max_step=1.0, min_step=0.15,
                        search_turn_time=1.5, max_motion_time=5.0)


def test_turn_left_when_target_far_left():
    step = control.decide(bearing=0.6, distance=2.0, tuning=TUNING)
    assert step.action == control.TURN
    assert step.angular_z == pytest.approx(0.4)
    assert step.linear_x == 0.0
    assert step.duration == pytest.approx(1.5)


def test_turn_right_when_target_far_right():
    step = control.decide(bearing=-0.8, distance=2.0, tuning=TUNING)
    assert step.action == control.TURN
    assert step.angular_z == pytest.approx(-0.4)
    assert step.duration == pytest.approx(2.0)


def test_walk_turns_toward_target():
    step = control.decide(bearing=0.3, distance=1.3, tuning=TUNING)
    assert step.action == control.WALK
    assert step.linear_x == pytest.approx(0.25)
    assert step.duration == pytest.approx(2.0)
    assert step.angular_z == pytest.approx(0.3 / 2.0)


def test_walk_turn_rate_capped():
    step = control.decide(bearing=-0.45, distance=0.85, tuning=TUNING)
    assert step.action == control.WALK
    assert step.angular_z == pytest.approx(-0.4)


def test_turn_before_done_when_close_but_misaligned():
    assert control.decide(bearing=0.3, distance=0.6, tuning=TUNING).action == control.TURN


def test_done_when_close_and_aligned():
    assert control.decide(bearing=0.05, distance=0.8, tuning=TUNING).action == control.DONE
    assert control.decide(bearing=-0.1, distance=0.5, tuning=TUNING).action == control.DONE


def test_walk_remaining_distance():
    step = control.decide(bearing=0.0, distance=1.3, tuning=TUNING)
    assert step.action == control.WALK
    assert step.linear_x == pytest.approx(0.25)
    assert step.angular_z == 0.0
    assert step.duration == pytest.approx(0.5 / 0.25)


def test_walk_capped_at_max_step():
    step = control.decide(bearing=0.0, distance=4.0, tuning=TUNING)
    assert step.duration == pytest.approx(1.0 / 0.25)


def test_walk_at_least_min_step():
    step = control.decide(bearing=0.0, distance=0.85, tuning=TUNING)
    assert step.duration == pytest.approx(0.15 / 0.25)


def test_walk_max_step_when_depth_unknown():
    step = control.decide(bearing=0.0, distance=None, tuning=TUNING)
    assert step.action == control.WALK
    assert step.duration == pytest.approx(1.0 / 0.25)


def test_motion_time_capped():
    slow = dataclasses.replace(TUNING, walk_speed=0.1, max_motion_time=3.0)
    assert control.decide(bearing=0.0, distance=4.0, tuning=slow).duration == pytest.approx(3.0)


def test_search_rotates_left():
    step = control.search_step(TUNING)
    assert step.action == control.SEARCH
    assert step.angular_z == pytest.approx(0.4)
    assert step.linear_x == 0.0
    assert step.duration == pytest.approx(1.5)


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
