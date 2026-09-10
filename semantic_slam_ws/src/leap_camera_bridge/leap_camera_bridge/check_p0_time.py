#!/usr/bin/env python3
"""check_p0_time.py -- offline sanity check for the P0 time-sync recording.

Usage:
    check_p0_time /path/to/rosbag2/dir [--topic-imu /imu]
    check_p0_time --live 10          # live check: subscribe for N seconds instead

Reads /imu and /camera/image_raw stamps from a rosbag2 (sqlite3) recording and
reports:
  - IMU rate, dt min/max, monotonic violations
  - Camera frame count / monotonic violations
  - Camera->IMU nearest-neighbor time offset stability (mean/std/min/max)

Exit code 0 on pass, 1 on failure.
"""

import sys

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, Imu


def ns_of(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def analyze(imu_stamps, cam_stamps, imu_topic, cam_topic):
    imu = np.asarray(imu_stamps, dtype=np.int64)
    cam = np.asarray(cam_stamps, dtype=np.int64)
    print('== P0 time-sync check ==')
    print('  %-28s n=%d' % (imu_topic, imu.size))
    print('  %-28s n=%d' % (cam_topic, cam.size))
    if imu.size == 0 or cam.size == 0:
        print('FAIL: missing data on either topic')
        return 1

    dt = np.diff(imu) / 1e6
    duration = (imu[-1] - imu[0]) / 1e9
    rate = (imu.size - 1) / duration if duration > 0 else 0.0
    imu_bad = int(np.sum(dt <= 0.0))
    print('  imu duration=%.1fs rate=%.1fHz dt(ms) min=%.2f mean=%.2f max=%.2f'
          % (duration, rate, dt.min(), dt.mean(), dt.max()))
    print('  imu non-monotonic (<=0ms) count = %d' % imu_bad)

    cam_dt = np.diff(cam) / 1e6
    cam_bad = int(np.sum(cam_dt <= 0.0))
    print('  cam frame dt(ms) min=%.1f mean=%.1f max=%.1f' % (
        cam_dt.min() if cam_dt.size else float('nan'),
        cam_dt.mean() if cam_dt.size else float('nan'),
        cam_dt.max() if cam_dt.size else float('nan')))
    print('  cam non-monotonic count = %d' % cam_bad)

    # Camera->IMU nearest neighbour offset, in ms.
    off = []
    idx = 0
    for c in cam:
        while idx + 1 < imu.size and abs(imu[idx + 1] - c) < abs(imu[idx] - c):
            idx += 1
        off.append((imu[idx] - c) / 1e6)
    off = np.asarray(off)
    print('  cam-imu nearest offset(ms) min=%.2f mean=%.2f std=%.2f max=%.2f' % (
        off.min(), off.mean(), off.std(), off.max()))
    print('  cam-imu offset jump>10ms count = %d' % int(np.sum(np.abs(np.diff(off)) > 10.0)))

    ok = imu_bad == 0 and cam_bad == 0 and rate >= 60.0 and rate <= 130.0
    print('RESULT: %s' % ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


def from_bag(bag_path, imu_topic, cam_topic):
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'),
    )
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    imu_stamps, cam_stamps = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == imu_topic and topics.get(topic, '').endswith('Imu'):
            imu_stamps.append(ns_of(deserialize_message(data, Imu)))
        elif topic == cam_topic and topics.get(topic, '').endswith('Image'):
            cam_stamps.append(ns_of(deserialize_message(data, Image)))
    return imu_stamps, cam_stamps


class _LiveNode(Node):
    def __init__(self, seconds, imu_topic, cam_topic):
        super().__init__('check_p0_time')
        self.seconds = seconds
        self.imu_stamps = []
        self.cam_stamps = []
        self._imu_topic = imu_topic
        self._cam_topic = cam_topic
        self.sub_imu = self.create_subscription(Imu, imu_topic, self._cb_imu, 100)
        self.sub_cam = self.create_subscription(Image, cam_topic, self._cb_cam, 10)

    def _cb_imu(self, msg):
        if ns_of(msg) > 0:
            self.imu_stamps.append(ns_of(msg))

    def _cb_cam(self, msg):
        if ns_of(msg) > 0:
            self.cam_stamps.append(ns_of(msg))

    def run(self):
        import time
        end = time.monotonic() + self.seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)


def main():
    args = [a for a in sys.argv[1:]]
    live = '--live' in args
    imu_topic = '/imu'
    cam_topic = '/camera/image_raw'
    if live:
        args.remove('--live')
        seconds = int(args[0]) if args else 10
        rclpy.init()
        node = _LiveNode(seconds, imu_topic, cam_topic)
        node.run()
        rc = analyze(node.imu_stamps, node.cam_stamps, imu_topic, cam_topic)
        node.destroy_node()
        rclpy.shutdown()
    else:
        if not args:
            print('usage: check_p0_time <rosbag2-dir> [--live N]')
            return 2
        imu, cam = from_bag(args[0], imu_topic, cam_topic)
        rc = analyze(imu, cam, imu_topic, cam_topic)
    sys.exit(rc)


if __name__ == '__main__':
    main()
