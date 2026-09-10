#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""semantic_localizer (P3): YOLO bottom pixel -> map coordinate.

Subscribes to /semantic/detections (P2 JSON).  For every detection it
undistorts the bbox bottom-center pixel, builds the camera ray, intersects
it with the ground plane (z=0 in base_footprint), and transforms the hit
point into the map frame using the TF tree
map -> odom -> base_footprint -> base_link  (P1).

Outputs:
  /semantic/objects_localized     String JSON (reliable)
  /semantic/localization_markers  MarkerArray  (reliable)
Optionally self-publishes /camera/camera_info (transient local) from a
Kalibr yaml when camera_info_source=file.
"""

import json
import os
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from xuegecar_localizer.localizer_math import (
    load_calib_yaml,
    pixel_to_base_point,
    rotation_base_optical,
    scale_intrinsics,
    transform_point,
)

RELIABLE = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)
INFO_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

CLASS_COLORS = {
    'bottle': (0.0, 0.9, 0.2),
    'person': (0.2, 0.5, 1.0),
    'cup': (1.0, 0.8, 0.0),
    'chair': (1.0, 0.5, 0.0),
}


def _round3(x):
    return round(float(x), 3)


class SemanticLocalizer(Node):

    def __init__(self):
        super().__init__('semantic_localizer')

        # ---- parameters ----------------------------------------------------
        self.declare_parameter('detections_topic', '/semantic/detections')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('camera_info_source', 'file')
        self.declare_parameter('camera_info_file', '')
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('objects_topic', '/semantic/objects_localized')
        self.declare_parameter('markers_topic', '/semantic/localization_markers')
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('ground_frame', 'base_footprint')
        self.declare_parameter('ground_z', 0.0)
        # 语义地面在 map 系下的报告高度（P4/RViz 约定 z=0=地面）；
        # 不受 P1 把 map/odom 平面锚在 base_link 高度(底盘+0.076)的影响
        self.declare_parameter('output_ground_z', 0.0)
        self.declare_parameter('camera_mount_x', 0.06)
        self.declare_parameter('camera_mount_y', 0.0)
        self.declare_parameter('camera_mount_z', 0.12)
        self.declare_parameter('camera_yaw_deg', 0.0)
        self.declare_parameter('camera_pitch_down_deg', 10.0)
        self.declare_parameter('camera_roll_deg', 0.0)
        self.declare_parameter('use_distortion', True)
        self.declare_parameter('auto_scale_intrinsics', True)
        self.declare_parameter('tf_timeout_s', 0.1)

        self.det_topic = str(self.get_parameter('detections_topic').value)
        self.info_topic = str(self.get_parameter('camera_info_topic').value)
        self.info_source = str(self.get_parameter('camera_info_source').value)
        self.info_file = str(self.get_parameter('camera_info_file').value)
        self.camera_frame_id = str(self.get_parameter('camera_frame_id').value)
        self.obj_topic = str(self.get_parameter('objects_topic').value)
        self.marker_topic = str(self.get_parameter('markers_topic').value)
        self.publish_markers = bool(self.get_parameter('publish_markers').value)
        self.output_frame = str(self.get_parameter('output_frame').value)
        self.ground_frame = str(self.get_parameter('ground_frame').value)
        self.ground_z = float(self.get_parameter('ground_z').value)
        self.output_ground_z = float(self.get_parameter('output_ground_z').value)
        self.use_distortion = bool(self.get_parameter('use_distortion').value)
        self.auto_scale_intrinsics = bool(
            self.get_parameter('auto_scale_intrinsics').value)
        self.tf_timeout = Duration(seconds=float(self.get_parameter('tf_timeout_s').value))

        mount = np.array([
            float(self.get_parameter('camera_mount_x').value),
            float(self.get_parameter('camera_mount_y').value),
            float(self.get_parameter('camera_mount_z').value),
        ])
        rot = rotation_base_optical(
            float(self.get_parameter('camera_yaw_deg').value),
            float(self.get_parameter('camera_pitch_down_deg').value),
            float(self.get_parameter('camera_roll_deg').value),
        )
        self._mount = mount
        self._rot = rot

        # ---- publishers / subscribers --------------------------------------
        self.pub_objects = self.create_publisher(String, self.obj_topic, RELIABLE)
        self.pub_markers = self.create_publisher(
            MarkerArray, self.marker_topic, RELIABLE)

        self._info_pub = None
        self._K = None          # np 3x3
        self._D = None          # np (5,)
        self._info_dims = None  # (w, h)
        self._have_info = False
        if self.info_source == 'topic':
            self.sub_info = self.create_subscription(
                CameraInfo, self.info_topic, self._on_camera_info, INFO_QOS)
        else:
            self._load_file_info(self.info_file)

        self.sub_det = self.create_subscription(
            String, self.det_topic, self._on_detections, RELIABLE)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # book-keeping
        self._n_msgs = 0
        self._n_objects = 0
        self._last_log = time.time()
        self._last_warn_no_info = 0.0
        self._last_warn_no_tf = 0.0
        self._scaled_k = {}          # (width, height) -> rescaled K
        self._scale_warned = set()   # resolutions already reported

        self.get_logger().info(
            'semantic_localizer started | det=%s | info_source=%s | '
            'obj=%s | marker=%s | mount=(%.3f, %.3f, %.3f) | '
            'pitch_down=%.1f deg | ground=%s z=%.2f | out_z=%.2f | out=%s'
            % (self.det_topic, self.info_source, self.obj_topic,
               self.marker_topic, mount[0], mount[1], mount[2],
               float(self.get_parameter('camera_pitch_down_deg').value),
               self.ground_frame, self.ground_z, self.output_ground_z,
               self.output_frame))

    # ------------------------------------------------------------------ info
    def _load_file_info(self, path):
        if not path or not os.path.exists(path):
            self.get_logger().error(
                'camera_info_file not found: %r - no intrinsics!' % path)
            return
        cal = load_calib_yaml(path)
        self._K = cal['K']
        self._D = cal['D']
        self._info_dims = (cal['width'], cal['height'])
        self._have_info = True
        msg = CameraInfo()
        msg.header.frame_id = self.camera_frame_id
        msg.height = cal['height']
        msg.width = cal['width']
        msg.distortion_model = cal['distortion_model']
        msg.d = list(self._D)
        k = self._K
        msg.k = [float(k[0, 0]), 0.0, float(k[0, 2]),
                 0.0, float(k[1, 1]), float(k[1, 2]),
                 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [float(k[0, 0]), 0.0, float(k[0, 2]), 0.0,
                 0.0, float(k[1, 1]), float(k[1, 2]), 0.0,
                 0.0, 0.0, 1.0, 0.0]
        self._info_pub = self.create_publisher(
            CameraInfo, self.info_topic, INFO_QOS)
        self._info_msg = msg
        self._info_pub.publish(msg)
        # transient-local on FastDDS does not always deliver a *once* sample to
        # late joiners -> republish periodically as well.
        self._info_timer = self.create_timer(5.0, self._republish_info)
        self.get_logger().info(
            'Camera intrinsics loaded from %s -> /%s | %.1f %.1f %.1f %.1f | %s'
            % (path, self.info_topic, cal['fx'], cal['fy'], cal['cx'],
               cal['cy'], cal['distortion_model']))

    def _republish_info(self):
        if self._info_pub is not None and self._info_msg is not None:
            self._info_pub.publish(self._info_msg)

    def _on_camera_info(self, msg):
        if msg.k and abs(msg.k[0]) > 1e-6 and abs(msg.k[4]) > 1e-6:
            self._K = np.array([[msg.k[0], 0.0, msg.k[2]],
                                [0.0, msg.k[4], msg.k[5]],
                                [0.0, 0.0, 1.0]])
            d = list(msg.d)[:5]
            while len(d) < 5:
                d.append(0.0)
            self._D = np.array(d, dtype=np.float64)
            self._info_dims = (msg.width, msg.height)
            self._have_info = True

    # ---------------------------------------------------------------- helpers
    def _intrinsics_for(self, width, height):
        """Return (K, scaled) adapted to the size of the incoming image.

        The calibration yaml is captured at ONE resolution (800x600 here),
        but the MJPEG stream can be configured to another one (these boards
        commonly serve 640x480 or 1280x720).  Feeding the calibration fx/cx
        to a differently sized frame mis-places every object -- laterally
        and in depth -- which shows up as "the bottle is straight ahead but
        the map says it is to the side".  The intrinsics are therefore
        rescaled here instead of failing silently.

        The radtan distortion coefficients are normalised (resolution
        independent), so D is left untouched.
        """
        if (not self.auto_scale_intrinsics or self._K is None
                or not self._info_dims or width <= 0 or height <= 0):
            return self._K, False
        cal_w, cal_h = int(self._info_dims[0]), int(self._info_dims[1])
        if width == cal_w and height == cal_h:
            return self._K, False
        key = (int(width), int(height))
        cached = self._scaled_k.get(key)
        if cached is not None:
            return cached, True

        scaled, sx, sy = scale_intrinsics(self._K, self._info_dims, key)
        if scaled is None or (sx == 1.0 and sy == 1.0):
            return self._K, False
        self._scaled_k[key] = scaled
        if key not in self._scale_warned:
            self._scale_warned.add(key)
            message = (
                'image %dx%d != calibration %dx%d -> intrinsics rescaled '
                'by (%.3f, %.3f): fx=%.1f cx=%.1f fy=%.1f cy=%.1f. '
                'Update camera_info_ov3660.yaml to the stream resolution '
                'to avoid this rescaling.'
                % (width, height, cal_w, cal_h, sx, sy,
                   scaled[0, 0], scaled[0, 2], scaled[1, 1], scaled[1, 2]))
            if abs(sx - sy) > 0.05 * max(sx, sy):
                # different aspect ratio: it is not a plain resize, the
                # rescale above is only an approximation
                self.get_logger().error(
                    message + ' WARNING: aspect ratio differs, this is a '
                    'crop/letterbox, not a resize - expect a scale error!')
            else:
                self.get_logger().warn(message)
        return scaled, True

    def _lookup_map_to_ground(self, sec, nsec):
        """Return map<-ground_frame transform at (sec,nsec); fallback latest."""
        for when in (Time(seconds=int(sec), nanoseconds=int(nsec)), Time()):
            try:
                return self._tf_buffer.lookup_transform(
                    self.output_frame, self.ground_frame, when,
                    timeout=self.tf_timeout)
            except Exception:
                continue
        return None

    # -------------------------------------------------------------- detections
    def _on_detections(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn('bad detections json: %s' % exc)
            return

        t0 = time.time()
        stamp = data.get('stamp') or {}
        sec = int(stamp.get('sec', 0))
        nsec = int(stamp.get('nanosec', 0))
        cam_frame = str(data.get('frame_id', self.camera_frame_id))
        width = int(data.get('image_width', 0))
        height = int(data.get('image_height', 0))
        dets = data.get('detections') or []
        # intrinsics actually used for this frame (rescaled if the stream
        # resolution differs from the calibration resolution)
        k_frame, k_scaled = self._intrinsics_for(width, height)

        objects = []
        markers = []
        marker_arr = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        delete_all.ns = 'semantic_localizer'
        marker_arr.markers.append(delete_all)

        n_map_ok = 0
        if not self._have_info:
            now = time.monotonic()
            if now - self._last_warn_no_info > 2.0:
                self._last_warn_no_info = now
                self.get_logger().warn(
                    'camera_info unavailable (source=%s); skipping projection'
                    % self.info_source)
        else:
            for i, d in enumerate(dets):
                obj = self._localize_one(d, i, sec, nsec, cam_frame, k_frame)
                if obj is None:
                    continue
                objects.append(obj['json'])
                if obj['map_point'] is not None:
                    n_map_ok += 1
                    markers.append(obj['marker_ground'])
                    markers.append(obj['marker_text'])

        for m in markers:
            marker_arr.markers.append(m)

        out = {
            'stamp': {'sec': sec, 'nanosec': nsec},
            'frame_id': self.output_frame,
            'camera_frame_id': cam_frame,
            'image_width': width,
            'image_height': height,
            # --- 现场自检用：内参是否被按分辨率缩放 ---
            'calib_width': (int(self._info_dims[0]) if self._info_dims else 0),
            'calib_height': (int(self._info_dims[1]) if self._info_dims else 0),
            'intrinsics_scaled': bool(k_scaled),
            'intrinsics_fx': (round(float(k_frame[0, 0]), 2)
                              if k_frame is not None else None),
            'intrinsics_cx': (round(float(k_frame[0, 2]), 2)
                              if k_frame is not None else None),
            'num_objects': len(objects),
            'objects': objects,
        }
        out_msg = String()
        out_msg.data = json.dumps(out, ensure_ascii=False)
        self.pub_objects.publish(out_msg)
        if self.publish_markers:
            self.pub_markers.publish(marker_arr)

        self._n_msgs += 1
        self._n_objects += len(objects)
        now = time.time()
        if now - self._last_log >= 5.0:
            per = 1000.0 * (now - t0)
            self.get_logger().info(
                'msg %d | objects %d | map_ok %d | %d ms/cb'
                % (self._n_msgs, len(objects), n_map_ok, per))
            self._n_msgs = 0
            self._n_objects = 0
            self._last_log = now

    def _localize_one(self, d, i, sec, nsec, cam_frame, k_frame=None):
        class_id = d.get('class_id')
        class_name = str(d.get('class_name', ''))
        confidence = float(d.get('confidence', 0.0))
        u = float(d.get('bottom_x', -1))
        v = float(d.get('bottom_y', -1))

        K = self._K if k_frame is None else k_frame
        base = pixel_to_base_point(
            u, v, K, self._D, self._mount, self._rot,
            ground_z=self.ground_z, use_distortion=self.use_distortion)
        obj = {
            'class_id': class_id,
            'class_name': class_name,
            'confidence': _round3(confidence),
            'u': int(round(u)),
            'v': int(round(v)),
            'position_base_footprint': None,
            'position_map': None,
            'distance_m': None,
            'ray_valid': False,
            'reason': None,
        }
        if base is None:
            obj['reason'] = 'ray_no_ground_hit (target above horizon / too close)'
            return {'json': obj, 'map_point': None, 'marker_ground': None,
                    'marker_text': None}
        obj['ray_valid'] = True
        obj['position_base_footprint'] = {
            'x': _round3(base[0]), 'y': _round3(base[1]), 'z': _round3(base[2])}
        obj['distance_m'] = _round3(float(np.hypot(base[0], base[1])))

        tr = self._lookup_map_to_ground(sec, nsec)
        map_point = None
        if tr is not None:
            pm = transform_point(tr, base)
            map_point = pm
            # x/y 取 TF 精确交点；z 归一为语义地面高度(默认0)。两平行水平面
            # 求交 x/y 完全一致，仅 z 平移，因此不影响 P3 的横向定位。
            obj['position_map'] = {
                'x': _round3(pm[0]), 'y': _round3(pm[1]),
                'z': _round3(self.output_ground_z)}
        else:
            now = time.monotonic()
            if now - self._last_warn_no_tf > 5.0:
                self._last_warn_no_tf = now
                self.get_logger().warn(
                    'TF %s<-%s unavailable at stamp' % (self.output_frame,
                                                        self.ground_frame))
            obj['reason'] = 'no_tf_%s_%s' % (self.output_frame, self.ground_frame)

        mg = mt = None
        if map_point is not None:
            mg = self._ground_marker(i, class_name, confidence, map_point, sec, nsec)
            mt = self._text_marker(i, class_name, confidence, map_point, sec, nsec)
        return {'json': obj, 'map_point': map_point,
                'marker_ground': mg, 'marker_text': mt}

    # ------------------------------------------------------------------ markers
    def _stamp_msg(self, sec, nsec):
        t = Time(seconds=int(sec), nanoseconds=int(nsec))
        return t.to_msg()

    def _ground_marker(self, i, name, conf, p, sec, nsec):
        color = CLASS_COLORS.get(name, (1.0, 1.0, 0.0))
        m = Marker()
        m.header.frame_id = self.output_frame
        m.header.stamp = self._stamp_msg(sec, nsec)
        m.ns = 'semantic_localizer'
        m.id = i * 2 + 1
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(p[0])
        m.pose.position.y = float(p[1])
        m.pose.position.z = 0.005
        m.scale.x = 0.10
        m.scale.y = 0.10
        m.scale.z = 0.01
        m.color.a = 0.9
        m.color.r, m.color.g, m.color.b = color
        return m

    def _text_marker(self, i, name, conf, p, sec, nsec):
        m = Marker()
        m.header.frame_id = self.output_frame
        m.header.stamp = self._stamp_msg(sec, nsec)
        m.ns = 'semantic_localizer'
        m.id = i * 2 + 2
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = float(p[0])
        m.pose.position.y = float(p[1])
        m.pose.position.z = 0.12
        m.scale.z = 0.10
        m.color.a = 1.0
        m.color.r, m.color.g, m.color.b = 1.0, 1.0, 1.0
        m.text = '%s %.2f' % (name, conf)
        return m


def main(args=None):
    rclpy.init(args=args)
    node = SemanticLocalizer()
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
