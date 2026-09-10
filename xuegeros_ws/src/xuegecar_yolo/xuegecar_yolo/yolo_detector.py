#!/usr/bin/env python3

"""xuegecar_yolo - YOLO detection node for the Leap physical car (P2).

Subscribes to /camera/image_raw (bgr8 sensor_msgs/Image published by
leap_camera_bridge) and publishes:

  /semantic/detections       String (JSON)  -- interface prepared for P3
  /semantic/image_annotated  Image (bgr8)   -- annotated image for viewing

Model: yolo26n.pt (80-class COCO). Frames are decoded with numpy directly,
so this node does NOT depend on cv_bridge (see module note in _decode_image).
"""

import json
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ultralytics import YOLO



def _decode_image(msg):
    """Decode a sensor_msgs/Image into a BGR uint8 ndarray.

    cv_bridge is intentionally avoided: on the robot PC the user-level python
    environment ships numpy 2.x + pip opencv-python, which is ABI-incompatible
    with the apt ros-humble-cv-bridge build (AttributeError: _ARRAY_API when
    importing cv_bridge). leap_camera_bridge publishes bgr8 with
    step = width * 3, so the fast path is a plain reshape.
    """
    encoding = (msg.encoding or '').lower()
    if encoding not in ('bgr8', 'rgb8'):
        raise ValueError('unsupported encoding: %r' % (msg.encoding,))
    if msg.height <= 0 or msg.width <= 0:
        raise ValueError('bad image size: %dx%d' % (msg.width, msg.height))
    if msg.step < msg.width * 3:
        raise ValueError('bad image step=%d (width=%d)' % (msg.step, msg.width))

    buf = np.frombuffer(msg.data, dtype=np.uint8)
    frame = buf.reshape((msg.height, msg.step))
    frame = frame[:, :msg.width * 3].copy()
    frame = frame.reshape((msg.height, msg.width, 3))

    if encoding == 'rgb8':
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame



def _encode_image(frame):
    """Build a bgr8 sensor_msgs/Image from a numpy frame (no cv_bridge)."""
    height, width = frame.shape[:2]
    msg = Image()
    msg.height = int(height)
    msg.width = int(width)
    msg.encoding = 'bgr8'
    msg.is_bigendian = False
    msg.step = int(width * 3)
    msg.data = np.ascontiguousarray(frame).tobytes()
    return msg



class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        # --- parameters -----------------------------------------------------
        self.declare_parameter(
            'model_path', '/home/chuyun/xuegeros_ws/models/yolo26n.pt')
        self.declare_parameter('model_label', '')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/semantic/detections')
        self.declare_parameter('annotated_topic', '/semantic/image_annotated')
        self.declare_parameter('confidence_threshold', 0.30)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('infer_every_n_frames', 1)
        self.declare_parameter('target_classes', ['bottle'])

        self.model_path = self._pstr('model_path')
        self.model_label = self._pstr('model_label')
        self.image_topic = self._pstr('image_topic')
        self.detections_topic = self._pstr('detections_topic')
        self.annotated_topic = self._pstr('annotated_topic')
        self.conf_threshold = (
            self.get_parameter('confidence_threshold')
            .get_parameter_value().double_value)
        self.iou_threshold = (
            self.get_parameter('iou_threshold')
            .get_parameter_value().double_value)
        self.image_size = (
            self.get_parameter('image_size')
            .get_parameter_value().integer_value)
        self.device = self._pstr('device')
        self.infer_every_n_frames = max(
            1, self.get_parameter('infer_every_n_frames')
            .get_parameter_value().integer_value)
        self.target_classes = set(
            self.get_parameter('target_classes')
            .get_parameter_value().string_array_value)

        if not self.model_label:
            base = os.path.basename(self.model_path)
            self.model_label = os.path.splitext(base)[0]

        # --- model ----------------------------------------------------------
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(
                'YOLO model not found: %s' % self.model_path)
        self.get_logger().info('Loading YOLO model: %s' % self.model_path)
        self.model = YOLO(self.model_path)
        self.get_logger().info(
            'Model %s loaded: task=%s, classes=%d'
            % (self.model_label, self.model.task, len(self.model.names)))

        # --- topics ---------------------------------------------------------
        self.detection_pub = self.create_publisher(
            String, self.detections_topic, 10)
        # Annotated images follow the image pipeline convention (best effort)
        # used by leap_camera_bridge; stale annotated frames may be dropped.
        self.annotated_pub = self.create_publisher(
            Image, self.annotated_topic, qos_profile_sensor_data)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback,
            qos_profile_sensor_data)

        self.frame_count = 0
        self.processed_frames = 0
        self.last_log_time = time.time()

        self.get_logger().info(
            'YOLO detector started | image=%s | detections=%s | annotated=%s | '
            'targets=%s | conf=%.2f | device=%s'
            % (self.image_topic, self.detections_topic, self.annotated_topic,
               ', '.join(sorted(self.target_classes)) or '*',
               self.conf_threshold, self.device))

    def _pstr(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.infer_every_n_frames != 0:
            return

        try:
            frame = _decode_image(msg)
        except Exception as exc:
            self.get_logger().warn('image decode failed: %s' % exc)
            return

        height, width = frame.shape[:2]
        start_time = time.time()
        try:
            result = self.model.predict(
                source=frame,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                imgsz=self.image_size,
                device=self.device,
                verbose=False)[0]
        except Exception as exc:
            self.get_logger().error('YOLO inference failed: %s' % exc)
            return
        inference_ms = (time.time() - start_time) * 1000.0

        detections = []
        boxes = getattr(result, 'boxes', None)
        if boxes is not None:
            for box in boxes:
                class_id = int(box.cls[0].item())
                confidence = float(box.conf[0].item())
                class_name = self.model.names.get(class_id, str(class_id))
                if (self.target_classes
                        and class_name not in self.target_classes):
                    continue

                xyxy = box.xyxy[0].cpu().numpy()
                x_min = int(round(xyxy[0]))
                y_min = int(round(xyxy[1]))
                x_max = int(round(xyxy[2]))
                y_max = int(round(xyxy[3]))
                x_min = max(0, min(width - 1, x_min))
                x_max = max(0, min(width - 1, x_max))
                y_min = max(0, min(height - 1, y_min))
                y_max = max(0, min(height - 1, y_max))

                center_x = (x_min + x_max) // 2
                center_y = (y_min + y_max) // 2
                bottom_x = center_x
                bottom_y = y_max

                detections.append({
                    'class_id': class_id,
                    'class_name': class_name,
                    'confidence': round(confidence, 4),
                    'x_min': x_min, 'y_min': y_min,
                    'x_max': x_max, 'y_max': y_max,
                    'center_x': center_x, 'center_y': center_y,
                    'bottom_x': bottom_x, 'bottom_y': bottom_y,
                })

                label = '%s %.2f' % (class_name, confidence)
                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max),
                              (0, 255, 0), 2)
                cv2.circle(frame, (bottom_x, bottom_y), 5,
                           (0, 0, 255), -1)
                cv2.putText(frame, label, (x_min, max(18, y_min - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        output = {
            'stamp': {
                'sec': int(msg.header.stamp.sec),
                'nanosec': int(msg.header.stamp.nanosec),
            },
            'frame_id': msg.header.frame_id,
            'image_width': width,
            'image_height': height,
            'model': self.model_label,
            'inference_time_ms': round(inference_ms, 2),
            'num_detections': len(detections),
            'detections': detections,
        }

        out_msg = String()
        out_msg.data = json.dumps(output, ensure_ascii=False)
        self.detection_pub.publish(out_msg)

        annotated_msg = _encode_image(frame)
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        self.processed_frames += 1
        now = time.time()
        if now - self.last_log_time >= 5.0:
            fps = self.processed_frames / (now - self.last_log_time)
            self.get_logger().info(
                'FPS: %.1f | infer: %.1f ms | detections: %d'
                % (fps, inference_ms, len(detections)))
            self.processed_frames = 0
            self.last_log_time = now



def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
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
