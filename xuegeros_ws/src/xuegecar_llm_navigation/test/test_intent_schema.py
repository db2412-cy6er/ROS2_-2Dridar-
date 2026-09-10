#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for the P5 LLM intent contract (no ROS, no network)."""

import pytest

from xuegecar_llm_navigation import intent_schema as schema


def test_system_prompt_contains_json_keyword_and_examples():
    prompt = schema.SYSTEM_PROMPT
    assert 'json' in prompt.lower()
    assert 'intent' in prompt
    assert 'nth_farthest' in prompt
    assert '{"intent"' in prompt.replace('\n', '') or 'intent' in prompt


def test_extract_plain_json():
    text = '{"intent": "navigate", "rank": 3}'
    assert schema.extract_json_object(text).startswith('{')


def test_extract_from_fenced_block():
    text = '```json\n{"intent": "query_count"}\n```'
    block = schema.extract_json_object(text)
    assert schema.normalize_plan(__import__('json').loads(block))[0][
        'intent'] == 'query_count'


def test_extract_from_surrounded_prose():
    text = '好的，结果如下：{"intent":"navigate","selection":"nearest"} 谢谢'
    assert schema.extract_json_object(text) == (
        '{"intent":"navigate","selection":"nearest"}')


def test_extract_with_nested_json():
    text = '{"intent":"navigate","extra":{"a":1},"rank":2}'
    raw = __import__('json').loads(schema.extract_json_object(text))
    assert raw['extra'] == {'a': 1}


def test_extract_raises_without_object():
    with pytest.raises(schema.IntentSchemaError):
        schema.extract_json_object('no json here')


def test_normalize_full_plan():
    plan, warnings = schema.normalize_plan({
        'intent': 'navigate',
        'object_class': 'Bottle',
        'spatial_filter': 'front',
        'selection': 'nth_farthest',
        'rank': 3,
        'object_id': '',
        'goal_relation': 'right',
        'goal_offset_m': 0.8,
    })
    assert plan['intent'] == 'navigate'
    assert plan['object_class'] == 'bottle'
    assert plan['rank'] == 3
    assert plan['goal_relation'] == 'right'
    assert plan['goal_offset_m'] == pytest.approx(0.8)
    assert warnings == []


def test_normalize_unknown_intent_and_bad_choice():
    plan, warnings = schema.normalize_plan({
        'intent': 'fly',
        'spatial_filter': 'up',
        'selection': 'third_farthest',
        'goal_relation': 'behind',
    })
    assert plan['intent'] == 'unknown'
    assert plan['spatial_filter'] == 'all'
    assert plan['selection'] == 'all'
    assert plan['goal_relation'] == 'near'
    assert len(warnings) == 4


def test_normalize_navigate_defaults_to_nearest():
    plan, _ = schema.normalize_plan({'intent': 'navigate'})
    assert plan['selection'] == 'nearest'


def test_normalize_rank_string_and_out_of_range():
    plan, _ = schema.normalize_plan({'intent': 'query_objects',
                                     'selection': 'nth_farthest',
                                     'rank': '3'})
    assert plan['rank'] == 3
    plan, warnings = schema.normalize_plan({'intent': 'query_objects',
                                            'selection': 'nth_farthest',
                                            'rank': 0})
    assert plan['rank'] == 1
    assert warnings


def test_normalize_offset_clamped():
    plan, warnings = schema.normalize_plan({'intent': 'navigate',
                                           'goal_offset_m': 0.01})
    assert plan['goal_offset_m'] == pytest.approx(schema.MIN_GOAL_OFFSET_M)
    assert warnings
    plan, _ = schema.normalize_plan({'intent': 'navigate',
                                    'goal_offset_m': 99})
    assert plan['goal_offset_m'] == pytest.approx(schema.MAX_GOAL_OFFSET_M)


def test_normalize_id_without_object_id_is_downgraded():
    plan, warnings = schema.normalize_plan(
        {'intent': 'navigate', 'selection': 'id', 'object_id': ''})
    assert plan['selection'] == 'nearest'
    assert warnings


def test_normalize_rejects_non_dict():
    with pytest.raises(schema.IntentSchemaError):
        schema.normalize_plan(['not', 'a', 'dict'])


def test_parse_plan_text_end_to_end():
    plan, _ = schema.parse_plan_text(
        '```json\n{"intent":"navigate","selection":"nth_farthest",'
        '"rank":3,"goal_relation":"right"}\n```')
    assert plan['intent'] == 'navigate'
    assert plan['selection'] == 'nth_farthest'
    assert plan['rank'] == 3
    assert plan['goal_relation'] == 'right'


def test_world_summary_is_compact():
    state = {
        'robot': {'x': 1.23456, 'y': -2.34567, 'yaw_deg': 91.234},
        'num_objects': 1,
        'objects': [{
            'id': 'bottle_001', 'class_name': 'bottle', 'x': 1.11111,
            'y': 2.22222, 'distance': 3.33333, 'relation': 'front',
            'visible_now': True,
        }],
    }
    summary = schema.world_summary(state)
    assert summary['robot']['x_map'] == pytest.approx(1.235)
    assert summary['objects'][0]['distance_to_robot_m'] == pytest.approx(
        3.333)
    assert summary['objects'][0]['relative_position'] == 'front'
