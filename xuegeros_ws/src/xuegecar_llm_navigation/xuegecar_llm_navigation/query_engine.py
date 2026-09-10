#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P5 deterministic semantic query engine.

Turns a P4 /semantic/dynamic_map snapshot plus a normalized intent plan
into deterministic facts, target selections and map-frame goal poses.

This module imports no ROS dependencies on purpose, so the whole
"deterministic half" of P5 can be unit tested without a robot, without
Nav2 and without network access.

Design rule: the LLM never computes anything.  Every number that reaches
an answer, or a Nav2 goal pose, is produced by the functions below.
"""

import math

SPATIAL_FILTERS = ('all', 'front', 'left', 'right', 'back')
SELECTIONS = ('all', 'nearest', 'farthest', 'nth_nearest',
              'nth_farthest', 'id')
GOAL_RELATIONS = ('near', 'left', 'right')

RELATION_CN = {
    'all': '',
    'front': '前方',
    'left': '左侧',
    'right': '右侧',
    'back': '后方',
}

GOAL_RELATION_CN = {'near': '旁边', 'left': '左边', 'right': '右边'}

#: robot pose and target must be at least this far apart to plan at all
MIN_TARGET_SEPARATION_M = 0.05

#: the goal must always stay this far in front of the robot
END_MARGIN_M = 0.05


# --------------------------------------------------------------------------
# small geometry helpers
# --------------------------------------------------------------------------

def normalize_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_quaternion(yaw):
    """Return (z, w) of the quaternion encoding a planar rotation."""
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def _as_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


# --------------------------------------------------------------------------
# map snapshot normalization
# --------------------------------------------------------------------------

def parse_map_snapshot(map_json):
    """Normalize a P4 dynamic_map dict into a deterministic state dict.

    Returns a dict with keys:
      ok, error, error_cn, robot, objects, num_objects, num_missing_distance
    ``error`` is None when ok, otherwise one of
      MAP_EMPTY / MAP_FORMAT / NO_ROBOT_POSE.
    """
    state = {
        'ok': False,
        'error': None,
        'error_cn': '',
        'robot': None,
        'objects': [],
        'num_objects': 0,
        'num_missing_distance': 0,
    }

    if not isinstance(map_json, dict) or not map_json:
        state['error'] = 'MAP_EMPTY'
        state['error_cn'] = (
            '当前没有收到有效的实时语义地图，请先确认 P4 正常运行。')
        return state

    raw_objects = map_json.get('objects')
    if raw_objects is None:
        raw_objects = []
    if not isinstance(raw_objects, list):
        state['error'] = 'MAP_FORMAT'
        state['error_cn'] = '实时语义地图格式不正确（objects 不是列表）。'
        return state

    robot = map_json.get('robot')
    if isinstance(robot, dict):
        rx = _as_float(robot.get('x_map'))
        ry = _as_float(robot.get('y_map'))
        ryaw = _as_float(robot.get('yaw_deg'))
        if rx is not None and ry is not None:
            state['robot'] = {
                'x': rx,
                'y': ry,
                'yaw_deg': ryaw if ryaw is not None else 0.0,
            }

    objects = []
    for obj in raw_objects:
        if not isinstance(obj, dict):
            continue
        position = obj.get('position_map')
        if not isinstance(position, dict):
            continue
        x = _as_float(position.get('x'))
        y = _as_float(position.get('y'))
        if x is None or y is None:
            continue
        distance = _as_float(obj.get('distance_to_robot_m'))
        relation = obj.get('relative_position')
        if relation not in SPATIAL_FILTERS:
            relation = None
        objects.append({
            'id': str(obj.get('id', '')),
            'class_name': str(obj.get('class_name', 'unknown')),
            'x': x,
            'y': y,
            'distance': distance,
            'relation': relation,
            'visible_now': bool(obj.get('visible_now', False)),
            'last_seen_age_sec': _as_float(obj.get('last_seen_age_sec')),
            'confidence': _as_float(obj.get('confidence')),
            'nearest_rank': obj.get('nearest_rank'),
            'farthest_rank': obj.get('farthest_rank'),
        })

    state['objects'] = objects
    state['num_objects'] = len(objects)
    state['num_missing_distance'] = sum(
        1 for obj in objects if obj['distance'] is None)
    state['ok'] = True
    return state


# --------------------------------------------------------------------------
# filtering / selection
# --------------------------------------------------------------------------

def filter_objects(objects, object_class=None, spatial_filter='all',
                   visible_only=False):
    """Filter normalized objects by class, spatial relation and visibility."""
    result = []
    for obj in objects:
        if object_class and obj.get('class_name') != object_class:
            continue
        if spatial_filter and spatial_filter != 'all':
            if obj.get('relation') != spatial_filter:
                continue
        if visible_only and not obj.get('visible_now'):
            continue
        result.append(obj)
    return result


def sort_by_distance(objects, reverse=False):
    """Sort objects that have a known distance (P5 never trusts upstream)."""
    valid = [obj for obj in objects if obj.get('distance') is not None]
    return sorted(valid, key=lambda obj: obj['distance'], reverse=reverse)


def select_object(objects, selection, rank=1, object_id=''):
    """Pick one object deterministically.

    Returns (object_or_None, reason) where reason is one of
      OK / NO_OBJECTS / NO_DISTANCE / ID_NOT_FOUND / RANK_OUT_OF_RANGE /
      SELECTION_ALL / UNKNOWN_SELECTION
    """
    if not objects:
        return None, 'NO_OBJECTS'

    if selection == 'id':
        wanted = str(object_id or '').strip()
        if not wanted:
            return None, 'ID_NOT_FOUND'
        for obj in objects:
            if obj.get('id') == wanted:
                return obj, 'OK'
        return None, 'ID_NOT_FOUND'

    if selection == 'all':
        return None, 'SELECTION_ALL'

    if selection not in ('nearest', 'farthest', 'nth_nearest',
                         'nth_farthest'):
        return None, 'UNKNOWN_SELECTION'

    if selection == 'nearest':
        ordered = sort_by_distance(objects)
        if not ordered:
            return None, 'NO_DISTANCE'
        return ordered[0], 'OK'

    if selection == 'farthest':
        ordered = sort_by_distance(objects, reverse=True)
        if not ordered:
            return None, 'NO_DISTANCE'
        return ordered[0], 'OK'

    ordered = sort_by_distance(objects, reverse=(selection == 'nth_farthest'))
    if not ordered:
        return None, 'NO_DISTANCE'
    try:
        index = int(rank) - 1
    except (TypeError, ValueError):
        index = 0
    if index < 0 or index >= len(ordered):
        return None, 'RANK_OUT_OF_RANGE'
    return ordered[index], 'OK'


def rank_cross_check(objects, selection, rank=1, object_id=''):
    """Compare P4's own rank fields with the locally computed ordering.

    P4 sorts with ``inf`` for objects whose distance is unknown, so its
    ``farthest_rank`` is only trustworthy when every object has a valid
    distance.  This helper returns a human readable warning string when
    the two disagree, otherwise None.  It is diagnostics only: P5 always
    uses its own ordering.
    """
    if selection not in ('nearest', 'farthest', 'nth_nearest',
                         'nth_farthest'):
        return None
    if any(obj.get('distance') is None for obj in objects):
        return ('存在无法测距的目标（定位未就绪），P4 排名字段可能不可信，'
                'P5 已改用本地排序。')
    ordered = sort_by_distance(objects, reverse=(selection == 'nth_farthest'))
    try:
        index = int(rank) - 1
    except (TypeError, ValueError):
        return None
    if index < 0 or index >= len(ordered):
        return None
    local = ordered[index]
    if selection == 'nth_farthest':
        upstream = local.get('farthest_rank')
    else:
        upstream = local.get('nearest_rank')
    if upstream is None:
        return None
    if int(upstream) != index + 1:
        return ('本地排序与 P4 排名字段不一致：本地第 %d 个是 %s，'
                'P4 字段为 %s。' % (index + 1, local.get('id'), upstream))
    return None


# --------------------------------------------------------------------------
# goal pose generation (frozen geometry, see P5 report §"右侧定义")
# --------------------------------------------------------------------------

def compute_goal(robot, target, relation, goal_offset_m,
                 min_approach_m=0.30, min_offset_m=0.28,
                 max_offset_m=2.0):
    """Build the map-frame goal pose for a target object.

    Frozen definition (robot -> bottle axis):
        u     = normalize(bottle - robot)
        right = ( u.y, -u.x)      left = (-u.y,  u.x)
        near  : goal = bottle - u     * eff
        right : goal = bottle + right * eff
        left  : goal = bottle + left  * eff
        yaw   = atan2(bottle.y - goal.y, bottle.x - goal.x)
    with eff = min(goal_offset_m, max(min_offset_m,
                                      centre_distance - END_MARGIN_M)).

    Safety invariant: ``min_offset_m`` (0.28 m) minus the Nav2
    ``xy_goal_tolerance`` (0.05 m in xuegebot_p5.yaml) must stay above
    robot 0.12 m + bottle 0.04 m + 0.05 m clearance = 0.21 m, so even the
    worst case arrival keeps the 5 cm surface clearance
    (0.28 - 0.05 = 0.23 >= 0.21).  The offset is the requested 0.30 m
    whenever the target is farther than ``min_approach_m`` (0.30 m);
    closer targets are reported as ALREADY_AT_TARGET.
    """
    result = {
        'ok': False,
        'reason': 'NO_POSE',
        'reason_cn': '无法根据当前语义地图生成导航目标。',
        'x': None,
        'y': None,
        'yaw': None,
        'eff_offset': None,
        'relation': relation,
        'centre_distance': None,
    }

    if not robot or not target:
        result['reason_cn'] = '缺少机器人位姿或目标信息，无法生成导航目标。'
        return result

    dx = target['x'] - robot['x']
    dy = target['y'] - robot['y']
    distance = math.hypot(dx, dy)

    if distance < MIN_TARGET_SEPARATION_M:
        result['reason'] = 'TARGET_TOO_CLOSE'
        result['reason_cn'] = '目标与机器人位置几乎重合，无法生成安全目标点。'
        result['centre_distance'] = distance
        return result

    if distance < min_approach_m:
        result['reason'] = 'ALREADY_AT_TARGET'
        result['reason_cn'] = (
            '目标距离机器人只有 %.2f 米，已经在旁边，无需移动。' % distance)
        result['centre_distance'] = distance
        return result

    eff = min(float(goal_offset_m), max(min_offset_m,
                                        distance - END_MARGIN_M))
    eff = max(min_offset_m, min(eff, max_offset_m))

    ux = dx / distance
    uy = dy / distance

    if relation == 'right':
        goal_x = target['x'] + uy * eff
        goal_y = target['y'] - ux * eff
    elif relation == 'left':
        goal_x = target['x'] - uy * eff
        goal_y = target['y'] + ux * eff
    else:
        relation = 'near'
        goal_x = target['x'] - ux * eff
        goal_y = target['y'] - uy * eff

    yaw = math.atan2(target['y'] - goal_y, target['x'] - goal_x)

    result.update({
        'ok': True,
        'reason': 'OK',
        'reason_cn': '',
        'x': goal_x,
        'y': goal_y,
        'yaw': normalize_angle(yaw),
        'eff_offset': eff,
        'relation': relation,
        'centre_distance': distance,
    })
    return result


def goal_relation_fallback(relation):
    """Ordered fallback list when a lateral goal is unreachable."""
    if relation == 'right':
        return ['near']
    if relation == 'left':
        return ['near']
    return []


# --------------------------------------------------------------------------
# deterministic fact strings (the LLM only gets to rephrase these)
# --------------------------------------------------------------------------

def describe_object(obj):
    if obj is None:
        return '未知目标'
    parts = [
        '%s：地图坐标(%.3f, %.3f)' % (obj['id'], obj['x'], obj['y']),
    ]
    if obj.get('distance') is not None:
        parts.append('距机器人 %.2f 米' % obj['distance'])
    if obj.get('relation'):
        parts.append('位于%s'
                     % RELATION_CN.get(obj['relation'], obj['relation']))
    text = '，'.join(parts)
    if not obj.get('visible_now'):
        age = obj.get('last_seen_age_sec')
        if age is not None:
            text += '（当前不在视野内，最后一次可见在 %.1f 秒前）' % age
        else:
            text += '（当前不在视野内）'
    return text


def fact_count(object_class, spatial_filter, count, num_missing=0):
    where = RELATION_CN.get(spatial_filter, '')
    fact = '当前小车%s的 %s 目标数量为 %d 个。' % (where, object_class, count)
    if num_missing:
        fact += '另有 %d 个目标因为定位信息缺失未能计入。' % num_missing
    return fact


def fact_objects_all(object_class, objects, num_missing=0):
    if not objects:
        return '当前没有检测到 %s 目标。' % object_class
    descriptions = [describe_object(obj) for obj in objects]
    summary = '当前共有 %d 个 %s 目标。%s'
    fact = summary % (len(objects), object_class, '；'.join(descriptions))
    if num_missing:
        fact += '另有 %d 个目标因为定位信息缺失未能列出。' % num_missing
    return fact


def fact_target(target, missing_reason=None):
    if target is None:
        return ('当前实时语义地图中没有满足该条件的目标。'
                + (('原因：%s' % missing_reason) if missing_reason else ''))
    return '目标是 %s。' % describe_object(target)


def fact_navigation(target, relation, goal, fallback_note=''):
    relation_cn = GOAL_RELATION_CN.get(relation, '旁边')
    fact = (
        '已选择 %s，准备导航到它的%s。导航目标地图坐标为(%.3f, %.3f)，'
        '与目标中心保持 %.2f 米。'
        % (target['id'], relation_cn, goal['x'], goal['y'],
           goal['eff_offset'])
    )
    if fallback_note:
        fact += fallback_note
    return fact


def fact_already_there(target, distance):
    return ('目标 %s 距离机器人只有 %.2f 米，已经在它旁边，无需导航。'
            % (target['id'], distance))


def fact_path_blocked(target, blocking_id, clearance_m, required_m):
    return (
        '没有执行导航：规划出的路径会经过 %s 附近，与 %s 的最小间隙只有 '
        '%.2f 米，低于安全阈值 %.2f 米，为避免撞到它已取消。'
        % (blocking_id, target['id'], clearance_m, required_m)
    )


def fact_goal_unreachable(target, relation):
    relation_cn = GOAL_RELATION_CN.get(relation, '旁边')
    return ('没有执行导航：%s 的%s 在当前代价地图中不可达（可能太靠近墙'
            '或障碍物）。' % (target['id'], relation_cn))
