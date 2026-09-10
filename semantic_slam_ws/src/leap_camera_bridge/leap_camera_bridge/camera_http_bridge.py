#!/usr/bin/env python3
# Copyright 2026 chuyun. MIT license.

"""camera_http_bridge: HTTP MJPEG stream -> ROS2 /camera/image_raw + /camera/camera_info.

Per-part headers emitted by the leap camera firmware (P0 time sync):
    X-Frame-Id
    X-Capture-Timestamp-Us   raw camera-local capture time (esp_timer us, since boot)
    X-Capture-Ros-Time-Ns    camera_local_us*1000 + ROSOFF  (0 while not synced)

Timestamp policy:
    - header.stamp is taken ONLY from X-Capture-Ros-Time-Ns.
    - If it is missing/0 (micro-ROS epoch not synchronized or mapping frozen-invalid),
      we publish stamp 0 and log a WARN once per second. We NEVER fall back to
      "PC receive time", because that would silently mix clock domains.
"""

import array
import threading
import time
import urllib.parse

import cv2
import numpy as np
import requests
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool

IMAGE_QOS = qos_profile_sensor_data  # best effort, depth 5, volatile
INFO_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class _MjpegParser:
    """Minimal buffered multipart/x-mixed-replace parser over a requests stream."""

    def __init__(self, response):
        self._response = response
        content_type = response.headers.get('Content-Type', '')
        parts = [p.strip() for p in content_type.split(';')]
        self._boundary = None
        for part in parts:
            if part.startswith('boundary='):
                self._boundary = part[len('boundary='):].strip('"').encode('ascii')
        if not self._boundary:
            raise RuntimeError('No boundary in Content-Type: %s' % content_type)
        self._marker = b'--' + self._boundary
        self._buf = bytearray()
        # urllib3 decodes HTTP chunked framing automatically on raw.read()
        self._response.raw.decode_content = True

    def _fill(self):
        try:
            chunk = self._response.raw.read(16384)
        except Exception:
            return False
        if not chunk:
            return False
        self._buf.extend(chunk)
        return True

    def _ensure(self, token, start=0):
        while True:
            idx = self._buf.find(token, start)
            if idx != -1:
                return idx
            if not self._fill():
                return -1

    @staticmethod
    def _parse_headers(header_bytes):
        headers = {}
        for line in header_bytes.split(b'\r\n'):
            if b':' not in line:
                continue
            key, _, value = line.partition(b':')
            headers[key.strip().lower().decode('ascii', 'replace')] = (
                value.strip().decode('ascii', 'replace')
            )
        return headers

    def read_part(self):
        """Blocking read of the next JPEG part.

        Returns (headers, jpeg_bytes) or None on stream end/error.
        """
        # Skip to the next boundary marker.
        idx = self._ensure(self._marker)
        if idx < 0:
            return None
        if idx > 0:
            del self._buf[:idx]

        # Consume the boundary marker line.
        eol = self._ensure(b'\r\n')
        if eol < 0:
            return None
        del self._buf[:eol + 2]

        # Parse part headers.
        hdr_end = self._ensure(b'\r\n\r\n')
        if hdr_end < 0:
            return None
        headers = self._parse_headers(bytes(self._buf[:hdr_end]))
        del self._buf[:hdr_end + 4]

        try:
            length = int(headers.get('content-length', '0'))
        except ValueError:
            length = 0
        if length <= 0:
            return None

        while len(self._buf) < length:
            if not self._fill():
                return None
        jpeg = bytes(self._buf[:length])
        del self._buf[:length]
        return headers, jpeg


def _ros_time_from_ns(ns):
    sec = int(ns // 1_000_000_000)
    nsec = int(ns % 1_000_000_000)
    return sec, nsec


class CameraHttpBridge(Node):

    def __init__(self):
        super().__init__('camera_http_bridge')

        self.declare_parameter('stream_url', 'http://192.168.31.100:81/')
        self.declare_parameter('config_url', '')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('camera_info_yaml', '')

        self.stream_url = self.get_parameter('stream_url').value
        self.frame_id = self.get_parameter('frame_id').value
        self.image_topic = self.get_parameter('image_topic').value
        self.info_topic = self.get_parameter('camera_info_topic').value
        self.info_yaml = self.get_parameter('camera_info_yaml').value

        raw_cfg = self.get_parameter('config_url').value
        if raw_cfg:
            self.config_url = raw_cfg
        else:
            parsed = urllib.parse.urlparse(self.stream_url)
            host = parsed.hostname or '192.168.31.100'
            self.config_url = 'http://%s/' % host
        self.config_url = self.config_url.rstrip('/')

        self.pub_image = self.create_publisher(Image, self.image_topic, IMAGE_QOS)
        self.pub_info = self.create_publisher(CameraInfo, self.info_topic, INFO_QOS)

        self._session = requests.Session()
        self._stop = threading.Event()
        self._last_warn = 0.0

        # camera_info (calibrated file optional; empty defaults before calibration)
        self.camera_info = self._build_camera_info()

        self.create_service(
            SetBool, '~/sync_hold', self._handle_sync_hold,
        )
        self._thread = threading.Thread(target=self._stream_loop, daemon=True)

    # ------------------------------------------------------------------ utils
    def _build_camera_info(self):
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width = 800
        info.height = 600
        info.distortion_model = ''
        info.d = array.array('d', [0.0] * 5)
        info.k = [0.0] * 9
        info.r = [0.0] * 9
        info.p = [0.0] * 12
        info.k[0] = info.k[4] = 1.0
        info.r[0] = info.r[4] = info.r[8] = 1.0
        info.p[0] = info.p[5] = info.p[10] = 1.0
        if not self.info_yaml:
            return info

        try:
            with open(self.info_yaml, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            info.width = int(data.get('width', info.width))
            info.height = int(data.get('height', info.height))
            info.distortion_model = str(data.get('distortion_model', ''))
            k = data.get('k') or data.get('camera_matrix')
            if isinstance(k, (list, tuple)) and len(k) >= 9:
                info.k = [float(v) for v in k[:9]]
            d = data.get('d') or data.get('distortion_coefficients')
            if isinstance(d, (list, tuple)):
                info.d = [float(v) for v in d]
            r = data.get('r') or data.get('rectification_matrix')
            if isinstance(r, (list, tuple)) and len(r) >= 9:
                info.r = [float(v) for v in r[:9]]
            p = data.get('p') or data.get('projection_matrix')
            if isinstance(p, (list, tuple)) and len(p) >= 12:
                info.p = [float(v) for v in p[:12]]
            self.get_logger().info(
                'Loaded calibrated camera_info: %dx%d model=%s'
                % (info.width, info.height, info.distortion_model))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('Failed to load camera_info yaml %s: %s' % (self.info_yaml, exc))
        return info

    def _publish_camera_info(self):
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_info.header.frame_id
        msg.width = self.camera_info.width
        msg.height = self.camera_info.height
        msg.distortion_model = self.camera_info.distortion_model
        msg.d = list(self.camera_info.d)
        msg.k = list(self.camera_info.k)
        msg.r = list(self.camera_info.r)
        msg.p = list(self.camera_info.p)
        self.pub_info.publish(msg)

    def _sync_endpoint(self, action):
        url = '%s/api/sync/%s' % (self.config_url, action)
        try:
            resp = self._session.get(url, timeout=3.0)
            resp.raise_for_status()
            self.get_logger().info('camera sync %s -> %s' % (action, resp.text.strip()))
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('camera sync %s failed: %s' % (action, exc))
            return False

    def _handle_sync_hold(self, request, response):
        ok = self._sync_endpoint('hold' if request.data else 'release')
        response.success = ok
        response.message = ('held' if request.data else 'released') if ok else 'failed'
        return response

    # ------------------------------------------------------------- streaming
    def _connect(self):
        return self._session.get(self.stream_url, stream=True, timeout=(5.0, 15.0))

    def _stream_loop(self):
        self.get_logger().info('camera_http_bridge started: %s' % self.stream_url)
        # Publish camera_info once at startup (transient-local keeps it for late subs).
        self._publish_camera_info()
        if self._sync_endpoint('status'):
            self.get_logger().info('Camera sync/status endpoint reachable at %s' % self.config_url)

        while not self._stop.is_set():
            try:
                with self._connect() as resp:
                    resp.raise_for_status()
                    parser = _MjpegParser(resp)
                    self.get_logger().info('MJPEG stream connected')
                    while not self._stop.is_set():
                        part = parser.read_part()
                        if part is None:
                            self.get_logger().warn('MJPEG stream ended; reconnecting...')
                            break
                        headers, jpeg = part
                        self._handle_part(headers, jpeg)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn('Stream error: %s (reconnect in 1s)' % exc)
            self._stop.wait(1.0)

    def _handle_part(self, headers, jpeg):
        try:
            ros_ns = int(headers.get('x-capture-ros-time-ns', '0') or 0)
            frame_id_raw = headers.get('x-frame-id', '')
        except ValueError:
            ros_ns, frame_id_raw = 0, ''

        if ros_ns <= 0:
            now = time.monotonic()
            if now - self._last_warn >= 1.0:
                self._last_warn = now
                self.get_logger().warn(
                    'X-Capture-Ros-Time-Ns invalid/0 (micro-ROS not synced or hold). '
                    'Publishing stamp 0; NOT using PC receive time.')
            sec, nsec = 0, 0
        else:
            sec, nsec = _ros_time_from_ns(ros_ns)

        img = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn('cv2.imdecode failed for frame %s' % frame_id_raw)
            return

        msg = Image()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nsec
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = img.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = int(msg.width * 3)
        msg.data = img.tobytes()
        self.pub_image.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraHttpBridge()
    node._thread.start()  # noqa: SLF001
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop.set()  # noqa: SLF001
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
