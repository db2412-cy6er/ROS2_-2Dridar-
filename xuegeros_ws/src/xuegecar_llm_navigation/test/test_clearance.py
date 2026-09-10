#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for the P5 clearance / obstacle-safety audit."""

import pytest

from xuegecar_llm_navigation import clearance as cl

ROBOT_RADIUS = 0.12
OBJECT_RADIUS = 0.04
MIN_CLEARANCE = 0.05


def obj(object_id, x, y):
    return {'id': object_id, 'x': x, 'y': y}


def make_costmap(width=20, height=20, resolution=0.1, origin_x=0.0,
                 origin_y=0.0, cells=()):
    data = [0] * (width * height)
    for mx, my, cost in cells:
        data[my * width + mx] = cost
    return {
        'resolution': resolution,
        'origin_x': origin_x,
        'origin_y': origin_y,
        'width': width,
        'height': height,
        'data': data,
    }


# ------------------------------------------------------------ point/segment

def test_point_segment_distance_perpendicular():
    assert cl.point_segment_distance(0.0, 1.0, 0.0, 0.0, 2.0, 0.0) == \
        pytest.approx(1.0)


def test_point_segment_distance_clamps_to_endpoints():
    assert cl.point_segment_distance(-1.0, 0.0, 0.0, 0.0, 2.0, 0.0) == \
        pytest.approx(1.0)
    assert cl.point_segment_distance(3.0, 0.0, 0.0, 0.0, 2.0, 0.0) == \
        pytest.approx(1.0)


def test_point_segment_distance_degenerate_segment():
    assert cl.point_segment_distance(1.0, 1.0, 0.0, 0.0, 0.0, 0.0) == \
        pytest.approx(2 ** 0.5)


# ---------------------------------------------------------------- path audit

def test_path_min_clearance_straight_through_object():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    report = cl.path_min_clearance(path, [obj('bottle_002', 1.0, 0.0)],
                                   ROBOT_RADIUS, OBJECT_RADIUS)
    assert report['centre_distance_m'] == pytest.approx(0.0)
    assert report['surface_clearance_m'] == pytest.approx(-0.16)
    assert report['blocking_id'] == 'bottle_002'


def test_path_min_clearance_beside_object():
    path = [(0.0, 0.0), (2.0, 0.0)]
    report = cl.path_min_clearance(path, [obj('bottle_002', 1.0, 0.25)],
                                   ROBOT_RADIUS, OBJECT_RADIUS)
    assert report['centre_distance_m'] == pytest.approx(0.25)
    assert report['surface_clearance_m'] == pytest.approx(0.09)


def test_path_min_clearance_ignores_listed_ids():
    path = [(0.0, 0.0), (2.0, 0.0)]
    report = cl.path_min_clearance(
        path, [obj('bottle_002', 1.0, 0.0)], ROBOT_RADIUS, OBJECT_RADIUS,
        ignore_ids=['bottle_002'])
    assert report['centre_distance_m'] is None


def test_audit_path_ok_when_clearance_is_enough():
    path = [(0.0, 0.0), (2.0, 0.0)]
    report = cl.audit_path(path, [obj('bottle_002', 1.0, 0.25)],
                           ROBOT_RADIUS, OBJECT_RADIUS, MIN_CLEARANCE)
    assert report['ok'] is True
    assert report['reason'] == 'OK'


def test_audit_path_blocks_low_bottle_invisible_to_lidar():
    """The whole point: Nav2 cannot see a short bottle, P5 must refuse."""
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    report = cl.audit_path(path, [obj('bottle_002', 1.0, 0.0)],
                           ROBOT_RADIUS, OBJECT_RADIUS, MIN_CLEARANCE)
    assert report['ok'] is False
    assert report['reason'] == 'SEMANTIC_CLEARANCE'
    assert report['blocking_id'] == 'bottle_002'


def test_audit_path_boundary_values():
    path = [(0.0, 0.0), (2.0, 0.0)]
    blocked = cl.audit_path(path, [obj('bottle_002', 1.0, 0.209)],
                            ROBOT_RADIUS, OBJECT_RADIUS, MIN_CLEARANCE)
    assert blocked['ok'] is False
    allowed = cl.audit_path(path, [obj('bottle_002', 1.0, 0.211)],
                            ROBOT_RADIUS, OBJECT_RADIUS, MIN_CLEARANCE)
    assert allowed['ok'] is True


def test_audit_path_empty():
    report = cl.audit_path([], [], ROBOT_RADIUS, OBJECT_RADIUS,
                           MIN_CLEARANCE)
    assert report['ok'] is False
    assert report['reason'] == 'EMPTY_PATH'


# ------------------------------------------------------------------ costmap

def test_costmap_index_lookup():
    assert cl.world_to_costmap_index(0.05, 0.05, 0.1, 0.0, 0.0, 20, 20) \
        == (0, 0)
    assert cl.world_to_costmap_index(1.05, 0.05, 0.1, 0.0, 0.0, 20, 20) \
        == (10, 0)
    assert cl.world_to_costmap_index(-0.5, 0.0, 0.1, 0.0, 0.0, 20, 20) \
        is None


def test_path_max_cost():
    costmap = make_costmap(cells=[(5, 0, 254)])
    report = cl.path_max_cost([(0.05, 0.05), (0.95, 0.05)], costmap)
    assert report['max_cost'] == 254
    assert 0.40 <= report['worst_point'][0] < 0.6


def test_path_max_cost_survives_float32_resolution_off_by_one():
    """0.05 arrives as 0.05000000074505806, shifting the lookup by a cell."""
    costmap = make_costmap(cells=[(11, 0, 254)])
    costmap['resolution'] = 0.05000000074505806
    report = cl.path_max_cost([(0.55, 0.05)], costmap)
    assert report['max_cost'] == 254


def test_path_max_cost_neighborhood_zero_is_exact():
    costmap = make_costmap(cells=[(11, 0, 254)])
    costmap['resolution'] = 0.05000000074505806
    report = cl.path_max_cost([(0.55, 0.05)], costmap, neighborhood=0)
    assert report['max_cost'] == 0


def test_densify_path_keeps_endpoints_and_fills_segments():
    dense = cl.densify_path([(0.0, 0.0), (1.0, 0.0)], 0.2)
    assert dense[0] == (0.0, 0.0)
    assert dense[-1] == (1.0, 0.0)
    assert len(dense) == 6
    assert cl.densify_path([(0.0, 0.0)], 0.2) == [(0.0, 0.0)]


def test_audit_path_blocks_lethal_costmap_cell():
    costmap = make_costmap(cells=[(5, 0, 254)])
    path = [(0.05, 0.05), (0.95, 0.05)]
    report = cl.audit_path(path, [], ROBOT_RADIUS, OBJECT_RADIUS,
                           MIN_CLEARANCE, costmap=costmap)
    assert report['ok'] is False
    assert report['reason'] == 'COSTMAP_LETHAL'


def test_audit_path_accepts_mild_costmap_cost():
    costmap = make_costmap(cells=[(5, 0, 199)])
    path = [(0.05, 0.05), (0.95, 0.05)]
    report = cl.audit_path(path, [], ROBOT_RADIUS, OBJECT_RADIUS,
                           MIN_CLEARANCE, costmap=costmap)
    assert report['ok'] is True
    assert report['max_cost'] == 199


# ---------------------------------------------------------------- watchdog

def test_classify_clearance_thresholds():
    kind, surface = cl.classify_clearance(0.50, ROBOT_RADIUS, OBJECT_RADIUS)
    assert kind == cl.CLEARANCE_OK
    assert surface == pytest.approx(0.34)

    kind, surface = cl.classify_clearance(0.20, ROBOT_RADIUS, OBJECT_RADIUS)
    assert kind == cl.CLEARANCE_WARN
    assert surface == pytest.approx(0.04)

    kind, _ = cl.classify_clearance(0.15, ROBOT_RADIUS, OBJECT_RADIUS)
    assert kind == cl.CLEARANCE_STOP

    kind, surface = cl.classify_clearance(None, ROBOT_RADIUS, OBJECT_RADIUS)
    assert kind == cl.CLEARANCE_OK
    assert surface is None


def test_nearest_object():
    objects = [obj('bottle_001', 3.0, 0.0), obj('bottle_002', 1.0, 0.0)]
    best, distance = cl.nearest_object(0.0, 0.0, objects)
    assert best['id'] == 'bottle_002'
    assert distance == pytest.approx(1.0)

    best, distance = cl.nearest_object(0.0, 0.0, [])
    assert best is None
    assert distance is None
