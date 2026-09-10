#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fake P4 dynamic-semantic-map publisher for offline P5 tests.

Publishes /semantic/dynamic_map with exactly the same QoS and JSON schema
as the real P4 node (depth 1, reliable, transient local, 5 Hz), so P5 can
be exercised without the camera, YOLO, P3 or the car.

The map content comes from a control file that may be rewritten while the
node runs (the file is re-read on every publish)::

    {
      "robot": {"x_map": 0.0, "y_map": 0.0, "yaw_deg": 0.0},
      "objects": [
        {"id": "bottle_001", "x": 0.8, "y": 0.0},
        {"id": "bottle_002", "x": 2.0, "y": 0.0}
      ],
      "remove": ["bottle_002"],
      "no_robot": false
    }

``distance_to_robot_m``, ``bearing_deg``, ``relative_position`` and the
rank fields are derived exactly like P4 does when they are omitted.
"""

import argparse
import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

DEFAULT_SPEC = {
    'robot': {'x_map': 0.0, 'y_map': 0.0, 'yaw_deg': 0.0},
    'objects': [
        {'id': 'bottle_001', 'x': 0.8, 'y': 0.0},
        {'id': 'bottle_002', 'x': 1.4, 'y': 0.0},
        {'id': 'bottle_003', 'x': 2.3, 'y': 0.0},
        {'id': 'bottle_004', 'x': 3.1, 'y': 0.0},
    ],
}


def relation_from_bearing(bearing_deg):
    if -45.0 <= bearing_deg <= 45.0:
        return 'front'
    if 45.0 < bearing_deg < 135.0:
        return 'left'
    if -135.0 < bearing_deg < -45.0:
        return 'right'
    return 'back'


def build_map(spec):
    """Turn a compact spec into the exact P4 JSON schema."""
    robot_spec = spec.get('robot') or {'x_map': 0.0, 'y_map': 0.0,
                                       'yaw_deg': 0.0}
    robot = None
    if not spec.get('no_robot', False):
        robot = {
            'x_map': float(robot_spec.get('x_map', 0.0)),
            'y_map': float(robot_spec.get('y_map', 0.0)),
            'yaw_deg': float(robot_spec.get('yaw_deg', 0.0)),
        }

    removed = set(spec.get('remove') or [])
    objects = []
    for item in (spec.get('objects') or []):
        if item.get('id') in removed:
            continue
        x = float(item['x'])
        y = float(item['y'])
        distance = item.get('distance')
        relation = item.get('relative_position')
        bearing_deg = None
        if robot is not None:
            dx = x - robot['x_map']
            dy = y - robot['y_map']
            if distance is None:
                distance = math.hypot(dx, dy)
            bearing_deg = math.degrees(
                math.atan2(dy, dx)) - robot['yaw_deg']
            bearing_deg = (bearing_deg + 180.0) % 360.0 - 180.0
            if relation is None:
                relation = relation_from_bearing(bearing_deg)
        objects.append({
            'id': str(item['id']),
            'class_id': int(item.get('class_id', 39)),
            'class_name': str(item.get('class_name', 'bottle')),
            'position_map': {'x': round(x, 4), 'y': round(y, 4), 'z': 0.0},
            'confidence': float(item.get('confidence', 0.9)),
            'observations': int(item.get('observations', 10)),
            'miss_count': 0,
            'visible_now': bool(item.get('visible_now', True)),
            'distance_to_robot_m': (None if distance is None
                                    else round(float(distance), 4)),
            'bearing_deg': (None if bearing_deg is None
                            else round(bearing_deg, 2)),
            'relative_position': relation,
            'last_seen_age_sec': float(item.get('last_seen_age_sec', 0.0)),
            'state': 'confirmed',
        })

    objects.sort(key=lambda obj: (float('inf')
                                  if obj['distance_to_robot_m'] is None
                                  else obj['distance_to_robot_m']))
    count = len(objects)
    for index, obj in enumerate(objects):
        obj['nearest_rank'] = index + 1
        obj['farthest_rank'] = count - index

    return {
        'stamp': 0.0,
        'frame_id': 'map',
        'robot': robot,
        'num_objects': len(objects),
        'objects': objects,
    }


class FakeDynamicMap(Node):
    """Publishes the fake map; the control file is re-read every cycle."""

    def __init__(self, control_file, rate):
        super().__init__('fake_dynamic_map')
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            String, '/semantic/dynamic_map', qos)
        self.control_file = control_file
        self.loaded_from_file = False
        self.create_timer(1.0 / max(0.5, rate), self.publish_map)
        self.get_logger().info(
            'fake P4 publisher started | control=%s rate=%.1fHz'
            % (control_file or '(built-in default)', rate))

    def _spec(self):
        if not self.control_file or not os.path.exists(self.control_file):
            return DEFAULT_SPEC
        try:
            with open(self.control_file, 'r', encoding='utf-8') as handle:
                spec = json.load(handle)
        except (OSError, ValueError) as exc:
            self.get_logger().warn('control file unreadable: %s' % exc)
            return DEFAULT_SPEC
        if not self.loaded_from_file:
            self.loaded_from_file = True
            self.get_logger().info('control file loaded: %s'
                                   % self.control_file)
        return spec

    def publish_map(self):
        data = build_map(self._spec())
        message = String()
        message.data = json.dumps(data, ensure_ascii=False)
        self.publisher.publish(message)


def main():
    parser = argparse.ArgumentParser(description='fake P4 map publisher')
    parser.add_argument('--control', default='',
                        help='json file with the map spec (re-read live)')
    parser.add_argument('--rate', type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = FakeDynamicMap(args.control, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
