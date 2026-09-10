#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P5 LLM intent contract (no ROS imports).

The language model is only allowed to translate one Chinese sentence into
this fixed JSON object.  Everything it returns is repaired, validated and
clamped here before any deterministic code sees it.

Contract::

    {"intent": "query_count|query_objects|navigate|cancel|unknown",
     "object_class": "bottle",
     "spatial_filter": "all|front|left|right|back",
     "selection": "all|nearest|farthest|nth_nearest|nth_farthest|id",
     "rank": 1,
     "object_id": "",
     "goal_relation": "near|left|right",
     "goal_offset_m": null}
"""

import json

INTENTS = ('query_count', 'query_objects', 'navigate', 'cancel', 'unknown')
SPATIAL_FILTERS = ('all', 'front', 'left', 'right', 'back')
SELECTIONS = ('all', 'nearest', 'farthest', 'nth_nearest', 'nth_farthest',
              'id')
GOAL_RELATIONS = ('near', 'left', 'right')

DEFAULT_OBJECT_CLASS = 'bottle'
MIN_GOAL_OFFSET_M = 0.20
MAX_GOAL_OFFSET_M = 2.00
MAX_RANK = 99

DEFAULT_PLAN = {
    'intent': 'unknown',
    'object_class': DEFAULT_OBJECT_CLASS,
    'spatial_filter': 'all',
    'selection': 'all',
    'rank': 1,
    'object_id': '',
    'goal_relation': 'near',
    'goal_offset_m': None,
}


class IntentSchemaError(ValueError):
    """Raised when the model output cannot be turned into a valid plan."""


SYSTEM_PROMPT = r"""P5_ROLE=intent_parser

你是室内移动机器人的高层语义指令解析器（intent parser）。

机器人维护一个实时动态语义地图 /semantic/dynamic_map，里面主要是 bottle。
你唯一的任务是把用户的一句话理解成结构化任务，**不允许计算坐标，
不允许编造任何数字或目标 ID**。

必须只输出一个合法的 json 对象（json, no markdown, no explanation）。

json 字段定义：
{
  "intent": "query_count | query_objects | navigate | cancel | unknown",
  "object_class": "bottle",
  "spatial_filter": "all | front | left | right | back",
  "selection": "all | nearest | farthest | nth_nearest | nth_farthest | id",
  "rank": 1,
  "object_id": "",
  "goal_relation": "near | left | right",
  "goal_offset_m": null
}

字段解释：
- intent：问数量=query_count；问位置/信息=query_objects；要求过去= navigate；
  要求停下/取消= cancel；听不懂= unknown。
- spatial_filter：只统计某个方位的目标时使用（前方 front / 左 left /
  右 right / 后 back），否则 all。
- selection：all=全部；nearest=最近；farthest=最远；
  nth_nearest=第 rank 近；nth_farthest=第 rank 远；id=按 object_id 指定。
- rank：配合 nth_* 使用，从 1 开始。
- object_id：只有用户明确说出某个 ID（如 bottle_002）时填写。
- goal_relation：near=旁边；left=左边；right=右边。
- goal_offset_m：用户明确要求停车距离（例如“停在半米外”）时填数字，
  否则 null。

参考例子：
1. “前方有几个瓶子” -> intent=query_count, spatial_filter=front
2. “瓶子都在哪里” -> intent=query_objects, selection=all
3. “最近的瓶子在哪里” -> intent=query_objects, selection=nearest
4. “第三远的瓶子在哪里” -> intent=query_objects, selection=nth_farthest,
   rank=3
5. “去最近的瓶子旁边” -> intent=navigate, selection=nearest,
   goal_relation=near
6. “去第三远的瓶子的右边” -> intent=navigate, selection=nth_farthest,
   rank=3, goal_relation=right
7. “去 bottle_002 左边” -> intent=navigate, selection=id,
   object_id=bottle_002, goal_relation=left
8. “停下 / 别去了 / 取消” -> intent=cancel

只输出 json，不要输出解释，不要输出 markdown 围栏。
"""


def build_user_prompt(question, world):
    """Build the user turn: the sentence plus the compact world state.

    The labels are deliberately ASCII: the offline mock server branches on
    them, and matching Chinese substrings across files turned out to be
    fragile (identical looking characters can differ in code points).
    """
    return (
        'P5_QUESTION=%s\n\nP5_WORLD=%s\n\n请输出 json。'
        % (question, json.dumps(world, ensure_ascii=False))
    )


def world_summary(state, max_objects=20):
    """Compact world state handed to the model (keeps tokens small)."""
    robot = state.get('robot')
    objects = []
    for obj in state.get('objects', [])[:max_objects]:
        objects.append({
            'id': obj['id'],
            'class_name': obj['class_name'],
            'x': round(obj['x'], 3),
            'y': round(obj['y'], 3),
            'distance_to_robot_m': (None if obj['distance'] is None
                                    else round(obj['distance'], 3)),
            'relative_position': obj['relation'],
            'visible_now': obj['visible_now'],
        })
    return {
        'robot': (None if robot is None else {
            'x_map': round(robot['x'], 3),
            'y_map': round(robot['y'], 3),
            'yaw_deg': round(robot['yaw_deg'], 1),
        }),
        'num_objects': state.get('num_objects', 0),
        'objects': objects,
    }


def extract_json_object(text):
    """Extract the first balanced {...} block, tolerating ```json fences."""
    if not isinstance(text, str):
        raise IntentSchemaError('model output is not text')
    cleaned = text.strip()
    if cleaned.startswith('```'):
        cleaned = cleaned.strip('`')
        if cleaned.lower().startswith('json'):
            cleaned = cleaned[4:]
    start = cleaned.find('{')
    if start < 0:
        raise IntentSchemaError('no json object found in model output')
    depth = 0
    in_string = False
    escaped = False
    for index in range(start, len(cleaned)):
        char = cleaned[index]
        if in_string:
            if escaped:
                escaped = False
            elif char == '\\':
                escaped = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
        elif char == '{':
            depth += 1
        elif char == '}':
            depth -= 1
            if depth == 0:
                return cleaned[start:index + 1]
    raise IntentSchemaError('unbalanced json object in model output')


def _coerce_choice(value, allowed, default):
    if isinstance(value, str):
        candidate = value.strip().lower()
        if candidate in allowed:
            return candidate, None
    return default, 'value %r not in %s, using %r' % (value, list(allowed),
                                                      default)


def _coerce_rank(value, default=1):
    try:
        rank = int(str(value).strip())
    except (TypeError, ValueError):
        return default, ('rank %r is not an integer, using %d'
                         % (value, default))
    if rank < 1 or rank > MAX_RANK:
        return default, ('rank %d out of range 1..%d, using %d'
                         % (rank, MAX_RANK, default))
    return rank, None


def _coerce_offset(value):
    if value is None or isinstance(value, bool):
        return None, None
    try:
        offset = float(value)
    except (TypeError, ValueError):
        return None, 'goal_offset_m %r is not a number, ignoring' % (value,)
    if offset < MIN_GOAL_OFFSET_M:
        return MIN_GOAL_OFFSET_M, ('goal_offset_m %.2f clamped to %.2f'
                                   % (offset, MIN_GOAL_OFFSET_M))
    if offset > MAX_GOAL_OFFSET_M:
        return MAX_GOAL_OFFSET_M, ('goal_offset_m %.2f clamped to %.2f'
                                   % (offset, MAX_GOAL_OFFSET_M))
    return offset, None


def normalize_plan(raw):
    """Validate/clamp a raw model dict into the frozen plan schema.

    Returns (plan, warnings).  Raises IntentSchemaError when ``raw`` is not
    a dict at all.
    """
    if not isinstance(raw, dict):
        raise IntentSchemaError('model output is not a json object')

    warnings = []
    plan = dict(DEFAULT_PLAN)

    intent, note = _coerce_choice(raw.get('intent', ''), INTENTS, 'unknown')
    plan['intent'] = intent
    if note:
        warnings.append(note)

    object_class = raw.get('object_class', DEFAULT_OBJECT_CLASS)
    if isinstance(object_class, str) and object_class.strip():
        plan['object_class'] = object_class.strip().lower()
    else:
        warnings.append('object_class missing, using %r'
                        % DEFAULT_OBJECT_CLASS)

    spatial, note = _coerce_choice(raw.get('spatial_filter', 'all'),
                                   SPATIAL_FILTERS, 'all')
    plan['spatial_filter'] = spatial
    if note:
        warnings.append(note)

    default_selection = ('nearest' if plan['intent'] == 'navigate' else 'all')
    selection, note = _coerce_choice(raw.get('selection', default_selection),
                                     SELECTIONS, default_selection)
    plan['selection'] = selection
    if note:
        warnings.append(note)

    plan['rank'], note = _coerce_rank(raw.get('rank', 1))
    if note:
        warnings.append(note)

    object_id = raw.get('object_id', '')
    plan['object_id'] = object_id.strip() if isinstance(object_id, str) else ''

    relation, note = _coerce_choice(raw.get('goal_relation', 'near'),
                                    GOAL_RELATIONS, 'near')
    plan['goal_relation'] = relation
    if note:
        warnings.append(note)

    plan['goal_offset_m'], note = _coerce_offset(raw.get('goal_offset_m'))
    if note:
        warnings.append(note)

    if plan['selection'] == 'id' and not plan['object_id']:
        warnings.append('selection=id but object_id is empty, '
                        'downgrading to nearest')
        plan['selection'] = 'nearest'

    return plan, warnings


def parse_plan_text(text):
    """Full pipeline: model text -> repaired json -> validated plan."""
    block = extract_json_object(text)
    try:
        raw = json.loads(block)
    except ValueError as exc:
        raise IntentSchemaError('json decode failed: %s' % exc)
    return normalize_plan(raw)


def phrase_system_prompt():
    """System prompt of the second call: rephrase a fact, never change it."""
    return (
        'P5_ROLE=phraser\n'
        '你是机器人语音回答模块。下面给你的 FACT 是机器人程序已经计算出的'
        '确定事实。必须严格根据 FACT 回答，不得改变其中的数字、目标 ID 和'
        '坐标，不得自己增加事实，不得省略安全提示。请用简短、自然的'
        '中文回答用户（不超过 60 字）。'
    )
