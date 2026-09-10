#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""P4 dynamic semantic mapper (xuegecar_dynamic_semantic_mapper).

Maintains a persistent set of confirmed semantic objects (bottle_001, ...)
from the per-frame P3 stream /semantic/objects_localized:

  /semantic/objects_localized (P3, incl. empty frames)
    |  same-class nearest-neighbour 1-1 data association
    |  min_hits consecutive detections  ->  confirmed (ADD event)
    v
  bottle_001 / bottle_002 ...   (EMA-refined map coordinates)
    |  visibility-gated negative observations: an unseen frame only counts as
    |  a miss when the object's old position currently lies inside the camera
    |  FOV band; delete_misses consecutive misses  ->  REMOVE event
    v
  /semantic/dynamic_map   (String JSON, transient local)
  /semantic/dynamic_markers
  /semantic/events
  services: /semantic/clear_dynamic_map, /semantic/save_dynamic_snapshot

Design corrections over the base spec:
  * tentative_max_age_s: never-confirmed tracks are purged by age so
    one-frame false positives cannot accumulate while the robot is turned
    away.  Confirmed tracks are NEVER removed by time - only by the
    visibility-gated misses (the core "do not delete what the camera does
    not currently see" rule).
  * same-frame same-class dedupe: two near-identical measurements of one
    bottle inside one P3 frame cannot create two tracks.  It uses its own
    (much smaller) gate ``same_frame_dedupe_distance`` so that two bottles
    standing ~20 cm apart in the same frame stay two objects.
"""

import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class DynamicSemanticMapper(Node):

    def __init__(self):
        super().__init__('dynamic_semantic_mapper')

        # ------------------------------------------------------- parameters
        for name, default in [
                ('input_topic', '/semantic/objects_localized'),
                ('camera_info_topic', '/camera/camera_info'),
                ('dynamic_map_topic', '/semantic/dynamic_map'),
                ('marker_topic', '/semantic/dynamic_markers'),
                ('event_topic', '/semantic/events'),
                ('map_frame', 'map'),
                ('robot_frame', 'base_footprint'),
                ('snapshot_file',
                 '/home/chuyun/xuegeros_ws/maps/p4_dynamic_snapshot.json'),
        ]:
            self.declare_parameter(name, default)

        for name, default in [
                ('association_distance', 0.15),
                ('same_frame_dedupe_distance', 0.05),
                ('position_alpha', 0.35),
                ('min_hits', 3),
                ('delete_misses', 8),
                ('tentative_delete_misses', 3),
                ('tentative_max_age_s', 30.0),
                ('min_confidence', 0.30),
                ('fallback_hfov_deg', 82.0),
                ('visibility_fov_scale', 0.80),
                ('min_visibility_distance', 0.25),
                ('max_visibility_distance', 2.50),
                ('camera_yaw_offset_deg', 0.0),
                ('publish_rate', 5.0),
        ]:
            self.declare_parameter(name, default)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.dynamic_map_topic = str(self.get_parameter('dynamic_map_topic').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)
        self.event_topic = str(self.get_parameter('event_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)

        self.association_distance = float(self.get_parameter('association_distance').value)
        self.same_frame_dedupe_distance = float(
            self.get_parameter('same_frame_dedupe_distance').value)
        self.position_alpha = float(self.get_parameter('position_alpha').value)
        self.min_hits = int(self.get_parameter('min_hits').value)
        self.delete_misses = int(self.get_parameter('delete_misses').value)
        self.tentative_delete_misses = int(
            self.get_parameter('tentative_delete_misses').value)
        self.tentative_max_age = float(self.get_parameter('tentative_max_age_s').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.hfov_rad = math.radians(
            float(self.get_parameter('fallback_hfov_deg').value))
        self.visibility_fov_scale = float(
            self.get_parameter('visibility_fov_scale').value)
        self.min_visibility_distance = float(
            self.get_parameter('min_visibility_distance').value)
        self.max_visibility_distance = float(
            self.get_parameter('max_visibility_distance').value)
        self.camera_yaw_offset = math.radians(
            float(self.get_parameter('camera_yaw_offset_deg').value))
        self.publish_rate = max(0.5, float(self.get_parameter('publish_rate').value))
        self.snapshot_file = os.path.expanduser(
            str(self.get_parameter('snapshot_file').value))

        # ------------------------------------------------------------- state
        self.tracks = {}          # id -> track dict
        self.class_counters = {}  # sanitized class -> running number
        self.last_robot_pose = None
        self._info_logged = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.input_sub = self.create_subscription(
            String, self.input_topic, self._on_input, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, 10)
        self.map_pub = self.create_publisher(
            String, self.dynamic_map_topic, transient_qos)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.marker_topic, transient_qos)
        self.event_pub = self.create_publisher(String, self.event_topic, 10)

        self.clear_service = self.create_service(
            Trigger, '/semantic/clear_dynamic_map', self._clear_callback)
        self.snapshot_service = self.create_service(
            Trigger, '/semantic/save_dynamic_snapshot', self._snapshot_callback)

        self.timer = self.create_timer(1.0 / self.publish_rate,
                                       self.publish_outputs)

        self.get_logger().info(
            'Dynamic semantic mapper started | in=%s out=%s | assoc=%.2f '
            'same_frame_dedupe=%.2f '
            'hits=%d delete=%d tent_delete=%d tent_age=%.1fs | fov_fallback=%.1f '
            'fov_scale=%.2f | dist=[%.2f, %.2f] | publish=%.1fHz'
            % (self.input_topic, self.dynamic_map_topic,
               self.association_distance, self.same_frame_dedupe_distance,
               self.min_hits, self.delete_misses,
               self.tentative_delete_misses, self.tentative_max_age,
               math.degrees(self.hfov_rad), self.visibility_fov_scale,
               self.min_visibility_distance, self.max_visibility_distance,
               self.publish_rate))

    # ------------------------------------------------------------ camera info
    def _on_camera_info(self, msg):
        if (msg.width > 0 and len(msg.k) >= 9 and msg.k[0] > 0.0):
            fx = float(msg.k[0])
            self.hfov_rad = 2.0 * math.atan(float(msg.width) / (2.0 * fx))
            if not self._info_logged:
                self._info_logged = True
                self.get_logger().info(
                    'camera_info %dx%d fx=%.2f -> HFOV=%.2f deg'
                    % (msg.width, msg.height, fx, math.degrees(self.hfov_rad)))

    # --------------------------------------------------------------- helpers
    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def distance(ax, ay, bx, by):
        return math.hypot(ax - bx, ay - by)

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_robot_pose(self):
        """(x, y, yaw) of robot_frame in map_frame, or None."""
        from rclpy.time import Time
        try:
            tf = self._tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, Time())
        except (TransformException, Exception):
            return None
        self.last_robot_pose = (
            float(tf.transform.translation.x),
            float(tf.transform.translation.y),
            self.quaternion_to_yaw(tf.transform.rotation),
        )
        return self.last_robot_pose

    @staticmethod
    def sanitize_class_name(class_name):
        safe = ''.join(c.lower() if (c.isalnum() or c == '_') else '_'
                       for c in str(class_name))
        return safe or 'object'

    def new_track_id(self, class_name):
        key = self.sanitize_class_name(class_name)
        self.class_counters[key] = self.class_counters.get(key, 0) + 1
        return '%s_%03d' % (key, self.class_counters[key])

    def publish_event(self, event_name, track):
        msg = String()
        msg.data = json.dumps({
            'event': event_name,
            'id': track['id'],
            'class_name': track['class_name'],
            'x_map': round(track['x'], 4),
            'y_map': round(track['y'], 4),
            'stamp': round(self.now_sec(), 3),
        }, ensure_ascii=False)
        self.event_pub.publish(msg)

    def track_visibility(self, track, robot_pose):
        """Return (visible, distance, bearing)."""
        if robot_pose is None:
            return False, None, None
        dx = track['x'] - robot_pose[0]
        dy = track['y'] - robot_pose[1]
        distance = math.hypot(dx, dy)
        camera_heading = robot_pose[2] + self.camera_yaw_offset
        bearing = self.normalize_angle(
            math.atan2(dy, dx) - camera_heading)
        half_fov = 0.5 * self.hfov_rad * self.visibility_fov_scale
        visible = (
            self.min_visibility_distance <= distance <= self.max_visibility_distance
            and abs(bearing) <= half_fov)
        return visible, distance, bearing

    @staticmethod
    def relation_from_bearing(bearing):
        if bearing is None:
            return None
        deg = math.degrees(bearing)
        if -45.0 <= deg <= 45.0:
            return 'front'
        if 45.0 < deg < 135.0:
            return 'left'
        if -135.0 < deg < -45.0:
            return 'right'
        return 'back'

    # --------------------------------------------------------------- processing
    def _on_input(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().error('input JSON parse failed: %s' % exc)
            return
        self._process_frame(data)

    def _process_frame(self, data):
        now = self.now_sec()
        robot_pose = self.get_robot_pose()

        raw = []
        for obj in data.get('objects') or []:
            if obj.get('ray_valid', True) is False:
                continue
            try:
                confidence = float(obj.get('confidence', 0.0))
            except Exception:
                continue
            if confidence < self.min_confidence:
                continue
            pos = obj.get('position_map')
            if not isinstance(pos, dict):
                continue
            try:
                x = float(pos['x'])
                y = float(pos['y'])
            except Exception:
                continue
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            raw.append({
                'class_id': int(obj.get('class_id', -1)),
                'class_name': str(obj.get('class_name', 'unknown')),
                'confidence': confidence,
                'x': x,
                'y': y,
            })

        # same-frame same-class dedupe (keep higher confidence measurement).
        # A *small* gate on purpose: two YOLO boxes inside one image really are
        # two objects (NMS already removed the duplicates of a single one),
        # while the 3D gate used for cross-frame association is much larger and
        # would merge two bottles standing ~20 cm apart.
        raw.sort(key=lambda m: -m['confidence'])
        measurements = []
        for m in raw:
            dup = any(
                c['class_name'] == m['class_name']
                and self.distance(m['x'], m['y'], c['x'], c['y'])
                <= self.same_frame_dedupe_distance
                for c in measurements)
            if not dup:
                measurements.append(m)

        # same-class nearest-neighbour, one-to-one data association
        candidates = []
        for mi, m in enumerate(measurements):
            for tid, tr in self.tracks.items():
                if tr['class_name'] != m['class_name']:
                    continue
                d = self.distance(m['x'], m['y'], tr['x'], tr['y'])
                if d <= self.association_distance:
                    candidates.append((d, mi, tid))
        candidates.sort(key=lambda item: item[0])

        used_m, used_t, assignments = set(), set(), []
        for _d, mi, tid in candidates:
            if mi in used_m or tid in used_t:
                continue
            used_m.add(mi)
            used_t.add(tid)
            assignments.append((mi, tid))

        # update matched tracks (EMA position, confirm on min_hits)
        for mi, tid in assignments:
            m = measurements[mi]
            tr = self.tracks[tid]
            a = self.position_alpha
            tr['x'] = (1.0 - a) * tr['x'] + a * m['x']
            tr['y'] = (1.0 - a) * tr['y'] + a * m['y']
            tr['confidence_sum'] += m['confidence']
            tr['observations'] += 1
            tr['hit_streak'] += 1
            tr['miss_count'] = 0
            tr['last_seen'] = now
            if not tr['confirmed'] and tr['hit_streak'] >= self.min_hits:
                tr['confirmed'] = True
                tr['confirmed_at'] = now
                self.publish_event('ADD', tr)

        # unmatched measurements -> new tentative tracks
        for mi, m in enumerate(measurements):
            if mi in used_m:
                continue
            tid = self.new_track_id(m['class_name'])
            self.tracks[tid] = {
                'id': tid,
                'class_id': m['class_id'],
                'class_name': m['class_name'],
                'x': m['x'],
                'y': m['y'],
                'confidence_sum': m['confidence'],
                'observations': 1,
                'hit_streak': 1,
                'miss_count': 0,
                'first_seen': now,
                'last_seen': now,
                'confirmed_at': None,
                'confirmed': False,
            }
            used_t.add(tid)

        # visibility-gated negative observations for unobserved tracks
        delete_ids = []
        for tid, tr in self.tracks.items():
            if tid in used_t:
                continue
            visible, _dist, _bearing = self.track_visibility(tr, robot_pose)
            if visible:  # old position is inside the FOV band -> real miss
                tr['miss_count'] += 1
                tr['hit_streak'] = 0
            threshold = (self.delete_misses if tr['confirmed']
                         else self.tentative_delete_misses)
            if tr['miss_count'] >= threshold:
                delete_ids.append(tid)

        for tid in delete_ids:
            tr = self.tracks.pop(tid)
            if tr['confirmed']:
                self.publish_event('REMOVE', tr)

        self.publish_outputs()

    # ------------------------------------------------------------ age cleanup
    def _purge_stale_tentative(self):
        """Remove never-confirmed tracks unseen for > tentative_max_age_s.

        Only tentative tracks (never entered the dynamic map) are touched;
        confirmed objects must persist while out of view by design.
        """
        now = self.now_sec()
        stale = [tid for tid, tr in self.tracks.items()
                 if not tr['confirmed']
                 and (now - tr['last_seen']) > self.tentative_max_age]
        for tid in stale:
            tr = self.tracks.pop(tid)
            self.publish_event('DISCARD', tr)

    # ------------------------------------------------------------- map output
    def build_dynamic_map(self):
        robot_pose = self.get_robot_pose()
        now = self.now_sec()

        objects = []
        for tr in self.tracks.values():
            if not tr['confirmed']:
                continue
            visible, distance, bearing = self.track_visibility(tr, robot_pose)
            if distance is None:
                bearing_deg = None
                relation = None
            else:
                bearing_deg = math.degrees(bearing)
                relation = self.relation_from_bearing(bearing)
            avg_conf = tr['confidence_sum'] / max(tr['observations'], 1)
            objects.append({
                'id': tr['id'],
                'class_id': tr['class_id'],
                'class_name': tr['class_name'],
                'position_map': {
                    'x': round(tr['x'], 4),
                    'y': round(tr['y'], 4),
                    'z': 0.0,
                },
                'confidence': round(avg_conf, 4),
                'observations': tr['observations'],
                'miss_count': tr['miss_count'],
                'visible_now': bool(visible),
                'distance_to_robot_m': (None if distance is None
                                        else round(distance, 4)),
                'bearing_deg': (None if bearing_deg is None
                                else round(bearing_deg, 2)),
                'relative_position': relation,
                'last_seen_age_sec': round(max(0.0, now - tr['last_seen']), 3),
                'state': 'confirmed',
            })

        def _sort_key(obj):
            d = obj['distance_to_robot_m']
            return float('inf') if d is None else d

        objects.sort(key=_sort_key)
        count = len(objects)
        for index, obj in enumerate(objects):
            obj['nearest_rank'] = index + 1
            obj['farthest_rank'] = count - index

        robot = None
        if robot_pose is not None:
            robot = {
                'x_map': round(robot_pose[0], 4),
                'y_map': round(robot_pose[1], 4),
                'yaw_deg': round(math.degrees(robot_pose[2]), 2),
            }

        return {
            'stamp': round(now, 3),
            'frame_id': self.map_frame,
            'robot': robot,
            'num_objects': len(objects),
            'objects': objects,
        }

    def publish_outputs(self):
        self._purge_stale_tentative()
        data = self.build_dynamic_map()

        map_msg = String()
        map_msg.data = json.dumps(data, ensure_ascii=False)
        self.map_pub.publish(map_msg)

        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = self.map_frame
        delete_all.header.stamp = self.get_clock().now().to_msg()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        marker_id = 1
        for obj in data['objects']:
            x = obj['position_map']['x']
            y = obj['position_map']['y']

            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'dynamic_semantic_objects'
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(x)
            sphere.pose.position.y = float(y)
            sphere.pose.position.z = 0.10
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.18
            sphere.scale.y = 0.18
            sphere.scale.z = 0.18
            sphere.color.r = 1.0
            sphere.color.g = 0.2
            sphere.color.b = 0.2
            sphere.color.a = 1.0
            marker_array.markers.append(sphere)

            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'dynamic_semantic_text'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(x)
            text.pose.position.y = float(y)
            text.pose.position.z = 0.32
            text.pose.orientation.w = 1.0
            text.scale.z = 0.16
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = '%s\nd=%sm\nmiss=%s' % (
                obj['id'],
                obj['distance_to_robot_m'],
                obj['miss_count'],
            )
            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)

    # ---------------------------------------------------------------- services
    def _clear_callback(self, request, response):
        for tr in list(self.tracks.values()):
            if tr['confirmed']:
                self.publish_event('REMOVE', tr)
        self.tracks = {}
        self.class_counters = {}
        self.publish_outputs()
        response.success = True
        response.message = 'Dynamic semantic map cleared.'
        return response

    def _snapshot_callback(self, request, response):
        try:
            data = self.build_dynamic_map()
            directory = os.path.dirname(self.snapshot_file)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self.snapshot_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            response.success = True
            response.message = 'Snapshot saved to: ' + self.snapshot_file
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DynamicSemanticMapper()
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
