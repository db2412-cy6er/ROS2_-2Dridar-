#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for the P5 deterministic query engine (no ROS, no network)."""

import math

import pytest

from xuegecar_llm_navigation import query_engine as qe


# ------------------------------------------------------------------ helpers

def make_object(object_id, x, y, class_name='bottle', distance=None,
                relation=None, visible=True, nearest_rank=None,
                farthest_rank=None):
    return {
        'id': object_id,
        'class_name': class_name,
        'position_map': {'x': x, 'y': y, 'z': 0.0},
        'distance_to_robot_m': distance,
        'relative_position': relation,
        'visible_now': visible,
        'last_seen_age_sec': 1.0,
        'confidence': 0.9,
        'nearest_rank': nearest_rank,
        'farthest_rank': farthest_rank,
    }


def make_map(objects, robot=(0.0, 0.0, 0.0)):
    return {
        'stamp': 1.0,
        'frame_id': 'map',
        'robot': (None if robot is None else {
            'x_map': robot[0], 'y_map': robot[1], 'yaw_deg': robot[2]}),
        'num_objects': len(objects),
        'objects': objects,
    }


FOUR_BOTTLES = [
    make_object('bottle_001', 0.8, 0.0, distance=0.8, relation='front',
                nearest_rank=1, farthest_rank=4),
    make_object('bottle_002', 2.3, 0.0, distance=2.3, relation='front',
                nearest_rank=3, farthest_rank=2),
    make_object('bottle_003', 1.4, 0.0, distance=1.4, relation='front',
                nearest_rank=2, farthest_rank=3),
    make_object('bottle_004', 3.1, 0.0, distance=3.1, relation='front',
                nearest_rank=4, farthest_rank=1),
]


# ------------------------------------------------------------------- parsing

def test_parse_map_empty():
    state = qe.parse_map_snapshot(None)
    assert state['ok'] is False
    assert state['error'] == 'MAP_EMPTY'


def test_parse_map_bad_objects():
    state = qe.parse_map_snapshot({'objects': 'nope'})
    assert state['ok'] is False
    assert state['error'] == 'MAP_FORMAT'


def test_parse_map_without_robot():
    state = qe.parse_map_snapshot(make_map(FOUR_BOTTLES, robot=None))
    assert state['ok'] is True
    assert state['robot'] is None
    assert state['num_objects'] == 4


def test_parse_map_normalizes_objects_and_missing_distance():
    state = qe.parse_map_snapshot(make_map([
        make_object('bottle_001', 1.0, 0.0, distance=1.0, relation='front'),
        make_object('bottle_002', 2.0, 0.0, distance=None, relation=None),
    ]))
    assert state['ok'] is True
    assert state['num_objects'] == 2
    assert state['num_missing_distance'] == 1
    assert state['objects'][1]['distance'] is None
    assert state['objects'][1]['relation'] is None


def test_parse_map_skips_invalid_entries():
    state = qe.parse_map_snapshot(make_map([
        make_object('bottle_001', 1.0, 0.0, distance=1.0),
        {'id': 'broken', 'position_map': {'x': 'abc', 'y': 0.0}},
        'not-a-dict',
    ]))
    assert [obj['id'] for obj in state['objects']] == ['bottle_001']


# ----------------------------------------------------------------- filtering

def test_filter_by_relation():
    objects = qe.parse_map_snapshot(make_map([
        make_object('bottle_001', 1.0, 0.0, relation='front'),
        make_object('bottle_002', -1.0, 0.0, relation='back'),
        make_object('bottle_003', 0.0, 1.0, relation='left'),
    ]))['objects']
    front = qe.filter_objects(objects, 'bottle', 'front')
    assert len(front) == 1
    assert front[0]['id'] == 'bottle_001'
    assert len(qe.filter_objects(objects, 'bottle', 'all')) == 3
    assert qe.filter_objects(objects, 'cup', 'all') == []


def test_filter_by_class_and_visibility():
    objects = qe.parse_map_snapshot(make_map([
        make_object('bottle_001', 1.0, 0.0, visible=True),
        make_object('bottle_002', 2.0, 0.0, visible=False),
        make_object('cup_001', 3.0, 0.0, class_name='cup', visible=True),
    ]))['objects']
    assert len(qe.filter_objects(objects, 'bottle', 'all',
                                 visible_only=True)) == 1
    assert len(qe.filter_objects(objects, None, 'all')) == 3


# ---------------------------------------------------------------- selection

@pytest.fixture
def four():
    return qe.parse_map_snapshot(make_map(FOUR_BOTTLES))['objects']


def test_select_nearest_and_farthest(four):
    nearest, reason = qe.select_object(four, 'nearest')
    assert reason == 'OK'
    assert nearest['id'] == 'bottle_001'

    farthest, reason = qe.select_object(four, 'farthest')
    assert reason == 'OK'
    assert farthest['id'] == 'bottle_004'


def test_select_nth_farthest_is_computed_locally(four):
    target, reason = qe.select_object(four, 'nth_farthest', rank=3)
    assert reason == 'OK'
    assert target['id'] == 'bottle_003'
    assert target['distance'] == 1.4


def test_select_nth_nearest(four):
    target, reason = qe.select_object(four, 'nth_nearest', rank=2)
    assert reason == 'OK'
    assert target['id'] == 'bottle_003'


def test_select_rank_out_of_range(four):
    target, reason = qe.select_object(four, 'nth_farthest', rank=9)
    assert target is None
    assert reason == 'RANK_OUT_OF_RANGE'


def test_select_rank_string_from_llm(four):
    target, reason = qe.select_object(four, 'nth_farthest', rank='3')
    assert reason == 'OK'
    assert target['id'] == 'bottle_003'


def test_select_by_id(four):
    target, reason = qe.select_object(four, 'id', object_id='bottle_002')
    assert reason == 'OK'
    assert target['id'] == 'bottle_002'
    target, reason = qe.select_object(four, 'id', object_id='bottle_099')
    assert target is None
    assert reason == 'ID_NOT_FOUND'


def test_select_ignores_objects_without_distance():
    objects = qe.parse_map_snapshot(make_map([
        make_object('bottle_001', 1.0, 0.0, distance=None),
        make_object('bottle_002', 2.0, 0.0, distance=2.0),
    ]))['objects']
    target, reason = qe.select_object(objects, 'nearest')
    assert reason == 'OK'
    assert target['id'] == 'bottle_002'


def test_select_no_distance_at_all():
    objects = qe.parse_map_snapshot(make_map([
        make_object('bottle_001', 1.0, 0.0, distance=None),
    ]))['objects']
    target, reason = qe.select_object(objects, 'nearest')
    assert target is None
    assert reason == 'NO_DISTANCE'


def test_select_empty_and_all(four):
    assert qe.select_object([], 'nearest') == (None, 'NO_OBJECTS')
    assert qe.select_object(four, 'all')[1] == 'SELECTION_ALL'
    assert qe.select_object(four, 'sideways')[1] == 'UNKNOWN_SELECTION'


def test_rank_cross_check(four):
    assert qe.rank_cross_check(four, 'nth_farthest', rank=3) is None
    broken = [dict(obj) for obj in four]
    broken[2]['farthest_rank'] = 1
    assert qe.rank_cross_check(broken, 'nth_farthest', rank=3) is not None


def test_rank_cross_check_flags_missing_distance():
    objects = qe.parse_map_snapshot(make_map([
        make_object('bottle_001', 1.0, 0.0, distance=None),
        make_object('bottle_002', 2.0, 0.0, distance=2.0),
    ]))['objects']
    assert qe.rank_cross_check(objects, 'nearest', rank=1) is not None


# ------------------------------------------------------------------ geometry

def _robot(x=0.0, y=0.0, yaw_deg=0.0):
    return {'x': x, 'y': y, 'yaw_deg': yaw_deg}


def _target(x, y, object_id='bottle_001'):
    return {'id': object_id, 'x': x, 'y': y, 'distance': None,
            'relation': None, 'visible_now': True}


def test_goal_near_is_between_robot_and_bottle():
    goal = qe.compute_goal(_robot(), _target(2.0, 0.0), 'near', 0.5)
    assert goal['ok'] is True
    assert goal['x'] == pytest.approx(1.5)
    assert goal['y'] == pytest.approx(0.0)
    assert goal['eff_offset'] == pytest.approx(0.5)
    assert goal['yaw'] == pytest.approx(0.0)


def test_goal_right_uses_robot_to_bottle_axis():
    goal = qe.compute_goal(_robot(), _target(2.0, 0.0), 'right', 0.5)
    assert goal['x'] == pytest.approx(2.0)
    assert goal['y'] == pytest.approx(-0.5)
    distance = math.hypot(goal['x'] - 2.0, goal['y'] - 0.0)
    assert distance == pytest.approx(0.5)
    cross = 1.0 * (goal['y'] - 0.0) - 0.0 * (goal['x'] - 2.0)
    assert cross < 0.0                      # negative cross = right side
    assert goal['yaw'] == pytest.approx(math.pi / 2.0)


def test_goal_left_uses_robot_to_bottle_axis():
    goal = qe.compute_goal(_robot(), _target(2.0, 0.0), 'left', 0.5)
    assert goal['y'] == pytest.approx(0.5)
    cross = 1.0 * (goal['y'] - 0.0) - 0.0 * (goal['x'] - 2.0)
    assert cross > 0.0                      # positive cross = left side
    assert goal['yaw'] == pytest.approx(-math.pi / 2.0)


def test_goal_right_follows_bottle_bearing():
    goal = qe.compute_goal(_robot(), _target(0.0, 2.0), 'right', 0.5)
    # u = (0, 1) -> right = (1, 0)
    assert goal['x'] == pytest.approx(0.5)
    assert goal['y'] == pytest.approx(2.0)
    assert goal['yaw'] == pytest.approx(math.pi)


def test_goal_offset_is_clamped_for_close_targets():
    goal = qe.compute_goal(_robot(), _target(0.45, 0.0), 'near', 0.5,
                           min_approach_m=0.40)
    assert goal['ok'] is True
    assert goal['eff_offset'] == pytest.approx(0.40)
    assert goal['x'] == pytest.approx(0.05)
    assert goal['x'] > 0.0                  # goal stays ahead of the robot


def test_goal_uses_requested_offset_for_reachable_target():
    goal = qe.compute_goal(_robot(), _target(0.80, 0.0), 'near', 0.5)
    assert goal['eff_offset'] == pytest.approx(0.5)
    assert goal['x'] == pytest.approx(0.30)


def test_goal_already_at_target():
    # Default threshold is 0.30 m: only closer targets are refused.  0.30 m
    # and 0.60 m MUST produce a goal -- the old 0.75 m default refused to move
    # at 0.6 m, which is the "it says it is already there" field bug
    # (see P5 report §10.1 / offline scenario S13b).
    goal = qe.compute_goal(_robot(), _target(0.20, 0.0), 'near', 0.30)
    assert goal['ok'] is False
    assert goal['reason'] == 'ALREADY_AT_TARGET'

    goal = qe.compute_goal(_robot(), _target(0.30, 0.0), 'near', 0.30)
    assert goal['ok'] is True
    assert goal['eff_offset'] == pytest.approx(0.28)
    goal = qe.compute_goal(_robot(), _target(0.60, 0.0), 'near', 0.30)
    assert goal['ok'] is True
    assert goal['eff_offset'] == pytest.approx(0.30)

    # the threshold is an explicit parameter: raising it refuses on purpose
    goal = qe.compute_goal(_robot(), _target(0.60, 0.0), 'near', 0.30,
                           min_approach_m=0.75)
    assert goal['ok'] is False
    assert goal['reason'] == 'ALREADY_AT_TARGET'


def test_goal_target_too_close():
    goal = qe.compute_goal(_robot(), _target(0.01, 0.0), 'near', 0.5)
    assert goal['ok'] is False
    assert goal['reason'] == 'TARGET_TOO_CLOSE'


def test_goal_without_robot_pose():
    goal = qe.compute_goal(None, _target(1.0, 0.0), 'near', 0.5)
    assert goal['ok'] is False
    assert goal['reason'] == 'NO_POSE'


def test_worst_case_nav2_tolerance_keeps_five_centimetres():
    robot_radius = 0.12
    object_radius = 0.04
    tolerance = 0.10                # xuegebot_p5.yaml xy_goal_tolerance
    for distance in (0.40, 0.45, 0.50, 0.60, 0.75, 1.00, 3.00):
        goal = qe.compute_goal(_robot(), _target(distance, 0.0),
                               'near', 0.5, min_approach_m=0.40)
        assert goal['ok'] is True
        worst_case = goal['eff_offset'] - tolerance
        assert worst_case - robot_radius - object_radius >= 0.05


def test_goal_relation_fallback():
    assert qe.goal_relation_fallback('right') == ['near']
    assert qe.goal_relation_fallback('left') == ['near']
    assert qe.goal_relation_fallback('near') == []


def test_yaw_to_quaternion():
    z, w = qe.yaw_to_quaternion(math.pi / 2.0)
    assert z == pytest.approx(math.sqrt(0.5))
    assert w == pytest.approx(math.sqrt(0.5))


# --------------------------------------------------------------------- facts

def test_fact_count_and_objects():
    assert '3' in qe.fact_count('bottle', 'front', 3)
    objects = qe.parse_map_snapshot(make_map(FOUR_BOTTLES))['objects']
    fact = qe.fact_objects_all('bottle', objects)
    assert 'bottle_003' in fact
    assert '4' in fact


def test_fact_target_none():
    fact = qe.fact_target(None, 'RANK_OUT_OF_RANGE')
    assert '没有满足' in fact


def test_fact_navigation_contains_goal():
    target = _target(1.4, 0.0, 'bottle_003')
    goal = qe.compute_goal(_robot(), target, 'right', 0.5)
    fact = qe.fact_navigation(target, 'right', goal)
    assert 'bottle_003' in fact
    assert '1.400, -0.500' in fact
    assert '0.50' in fact
