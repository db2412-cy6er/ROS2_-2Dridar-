#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P5 clearance / obstacle-safety audit (no ROS imports).

All distances are in metres.  The safety rule the whole project uses is:

    surface clearance = centre distance - robot_physical_radius
                                       - assumed_object_radius

and it must stay >= ``min_surface_clearance_m`` (default 0.05 m = 5 cm).

``robot_physical_radius`` is the real circumscribed radius of the car
(0.12 m: chassis 0.10 m plus the wheels sticking out to +-0.12 m, see
xuegecar.urdf), NOT the Nav2 ``robot_radius`` (which already contains the
safety margin).
"""

import math

CLEARANCE_OK = 'OK'
CLEARANCE_WARN = 'WARN'
CLEARANCE_STOP = 'STOP'

COSTMAP_LETHAL_COST = 253


def point_segment_distance(px, py, ax, ay, bx, by):
    """Shortest distance from point P to segment AB."""
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / length_sq
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def path_min_clearance(path_points, objects, robot_physical_radius,
                       assumed_object_radius, ignore_ids=()):
    """Minimum centre/surface distance between a path and every object.

    ``path_points`` is a list of (x, y); each consecutive pair is a
    segment, so the audit is exact in continuous metres and does not
    suffer from the 0.05 m costmap quantisation.

    Returns a dict with centre_distance_m, surface_clearance_m,
    blocking_id (None when there are no objects to check).
    """
    result = {
        'centre_distance_m': None,
        'surface_clearance_m': None,
        'blocking_id': None,
    }
    if len(path_points) < 2 or not objects:
        return result

    best_centre = None
    best_id = None
    for obj in objects:
        if obj.get('id') in ignore_ids:
            continue
        obj_best = None
        for index in range(len(path_points) - 1):
            ax, ay = path_points[index]
            bx, by = path_points[index + 1]
            distance = point_segment_distance(obj['x'], obj['y'],
                                              ax, ay, bx, by)
            if obj_best is None or distance < obj_best:
                obj_best = distance
        if obj_best is None:
            continue
        if best_centre is None or obj_best < best_centre:
            best_centre = obj_best
            best_id = obj.get('id')

    if best_centre is None:
        return result

    result['centre_distance_m'] = best_centre
    result['surface_clearance_m'] = (
        best_centre - robot_physical_radius - assumed_object_radius)
    result['blocking_id'] = best_id
    return result


def world_to_costmap_index(x, y, resolution, origin_x, origin_y,
                           width, height):
    """Return (mx, my) grid indices or None when outside the grid."""
    if resolution is None or resolution <= 0.0:
        return None
    mx = int(math.floor((x - origin_x) / resolution))
    my = int(math.floor((y - origin_y) / resolution))
    if mx < 0 or my < 0 or mx >= width or my >= height:
        return None
    return mx, my


def densify_path(path_points, step):
    """Insert intermediate samples so no cell is skipped between poses.

    NavFn publishes roughly one pose per cell, but the fake planner and
    future planners may not, and a sparse path would let the costmap
    audit miss a lethal cell that lies between two poses.
    """
    if len(path_points) < 2 or step <= 0.0:
        return list(path_points)
    dense = [path_points[0]]
    for index in range(len(path_points) - 1):
        ax, ay = path_points[index]
        bx, by = path_points[index + 1]
        length = math.hypot(bx - ax, by - ay)
        if length <= 1e-9:
            continue
        count = int(math.ceil(length / step))
        for k in range(1, count + 1):
            t = float(k) / float(count)
            dense.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return dense


def path_max_cost(path_points, costmap, neighborhood=1):
    """Highest costmap cell cost touched by the path.

    ``costmap`` is a dict: resolution, origin_x, origin_y, width, height
    and data (row major, bottom-up, as in nav_msgs/OccupancyGrid).

    Two robustness measures, both needed in practice:

    * the path is densified to half a cell first, so a lethal cell sitting
      between two poses is still detected;
    * every sample is expanded to a (2*neighborhood+1)^2 block.  The
      resolution arrives as a float32 (0.05000000074505806 in practice), so
      the same world point can map to a neighbouring cell in the writer and
      in the reader; a single-cell lookup silently misses obstacles.

    Returns a dict with max_cost and worst_point; max_cost is None when no
    path point falls inside the grid.
    """
    result = {'max_cost': None, 'worst_point': None, 'outside_points': 0}
    if not costmap or not path_points:
        return result

    data = costmap.get('data') or []
    resolution = costmap.get('resolution')
    width = costmap.get('width') or 0
    height = costmap.get('height') or 0
    origin_x = costmap.get('origin_x') or 0.0
    origin_y = costmap.get('origin_y') or 0.0

    step = None
    if resolution:
        step = max(0.5 * float(resolution), 0.01)
    samples = densify_path(path_points, step) if step else list(path_points)

    worst = None
    worst_point = None
    outside = 0
    reach = max(0, int(neighborhood))
    for x, y in samples:
        index = world_to_costmap_index(x, y, resolution, origin_x,
                                       origin_y, width, height)
        if index is None:
            outside += 1
            continue
        mx, my = index
        for offset_x in range(-reach, reach + 1):
            for offset_y in range(-reach, reach + 1):
                cx = mx + offset_x
                cy = my + offset_y
                if cx < 0 or cy < 0 or cx >= width or cy >= height:
                    continue
                flat = cy * width + cx
                if flat < 0 or flat >= len(data):
                    continue
                cost = int(data[flat])
                if worst is None or cost > worst:
                    worst = cost
                    worst_point = (x, y)

    result['max_cost'] = worst
    result['worst_point'] = worst_point
    result['outside_points'] = outside
    return result


def audit_path(path_points, objects, robot_physical_radius,
               assumed_object_radius, min_surface_clearance_m,
               costmap=None, max_cell_cost=COSTMAP_LETHAL_COST,
               ignore_ids=()):
    """Decide whether a planned path is safe to execute.

    Returns a dict with ok, reason, reason_cn, min_surface_clearance_m,
    min_centre_distance_m, blocking_id and max_cost.

    reasons: OK / EMPTY_PATH / SEMANTIC_CLEARANCE / COSTMAP_LETHAL
    """
    report = {
        'ok': False,
        'reason': 'OK',
        'reason_cn': '',
        'min_surface_clearance_m': None,
        'min_centre_distance_m': None,
        'blocking_id': None,
        'max_cost': None,
    }

    if not path_points or len(path_points) < 2:
        report['reason'] = 'EMPTY_PATH'
        report['reason_cn'] = '规划器没有返回有效路径。'
        return report

    semantic = path_min_clearance(path_points, objects,
                                  robot_physical_radius,
                                  assumed_object_radius, ignore_ids)
    report['min_surface_clearance_m'] = semantic['surface_clearance_m']
    report['min_centre_distance_m'] = semantic['centre_distance_m']
    report['blocking_id'] = semantic['blocking_id']

    if (semantic['surface_clearance_m'] is not None
            and semantic['surface_clearance_m'] < min_surface_clearance_m):
        report['reason'] = 'SEMANTIC_CLEARANCE'
        report['reason_cn'] = (
            '路径与 %s 的最小表面间隙 %.3f 米，低于阈值 %.3f 米。'
            % (semantic['blocking_id'],
               semantic['surface_clearance_m'],
               min_surface_clearance_m))
        return report

    if costmap:
        cost_report = path_max_cost(path_points, costmap)
        report['max_cost'] = cost_report['max_cost']
        if (cost_report['max_cost'] is not None
                and cost_report['max_cost'] >= max_cell_cost):
            report['reason'] = 'COSTMAP_LETHAL'
            report['reason_cn'] = (
                '路径穿过代价地图的致命/内切代价区（最大代价 %d）。'
                % cost_report['max_cost'])
            return report

    report['ok'] = True
    return report


def classify_clearance(centre_distance, robot_physical_radius,
                       assumed_object_radius, warn_surface_m=0.05,
                       stop_surface_m=0.0):
    """Runtime watchdog classification for the current robot-object gap."""
    if centre_distance is None:
        return CLEARANCE_OK, None
    surface = (centre_distance - robot_physical_radius
               - assumed_object_radius)
    if surface < stop_surface_m:
        return CLEARANCE_STOP, surface
    if surface < warn_surface_m:
        return CLEARANCE_WARN, surface
    return CLEARANCE_OK, surface


def nearest_object(robot_x, robot_y, objects, ignore_ids=()):
    """Return (object, centre_distance) of the closest object."""
    best = None
    best_distance = None
    for obj in objects:
        if obj.get('id') in ignore_ids:
            continue
        distance = math.hypot(obj['x'] - robot_x, obj['y'] - robot_y)
        if best_distance is None or distance < best_distance:
            best = obj
            best_distance = distance
    return best, best_distance
