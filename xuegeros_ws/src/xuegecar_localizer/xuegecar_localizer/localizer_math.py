#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""P3 pure geometry helpers (no ROS types) for semantic_localizer.

Conventions:
  base frame      : REP-103, x forward / y left / z up; ground plane z = ground_z
  camera (optical): +z forward / +x right / +y down (image convention)
  yaw/pitch/roll  : applied as base-frame extrinsic rotations on the rigid
                    camera body.  camera_pitch_down_deg > 0 means the optical
                    axis is pitched DOWN (looks at the ground).
"""

import cv2
import numpy as np
import yaml


def deg2rad(x):
    return float(x) * np.pi / 180.0


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, c, -s],
                     [0.0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s],
                     [0.0, 1.0, 0.0],
                     [-s, 0.0, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0],
                     [s, c, 0.0],
                     [0.0, 0.0, 1.0]])


def rotation_base_optical(yaw_deg=0.0, pitch_down_deg=0.0, roll_deg=0.0):
    """R (base <- optical).  Columns are the optical axes expressed in base."""
    # level, yaw=0 camera: right=(0,-1,0) down=(0,0,-1) fwd=(1,0,0)
    level = np.array([[0.0, 0.0, 1.0],
                      [-1.0, 0.0, 0.0],
                      [0.0, -1.0, 0.0]])
    q = (rot_z(deg2rad(yaw_deg))
         @ rot_y(deg2rad(pitch_down_deg))
         @ rot_x(deg2rad(roll_deg)))
    return q @ level


def rot_from_quat(q):
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_point(transform_msg, point):
    """Apply a geometry_msgs/Transform (with header-less accessors) to a 3-vector."""
    body = transform_msg.transform  # lookup_transform() returns TransformStamped
    t = body.translation
    r = body.rotation
    R = rot_from_quat((r.x, r.y, r.z, r.w))
    p = np.asarray(point, dtype=np.float64)
    return R @ p + np.array([t.x, t.y, t.z], dtype=np.float64)


def scale_intrinsics(K, cal_dims, image_dims):
    """Rescale a camera matrix from the calibration resolution to the image.

    The calibration yaml has one resolution (800x600 for this camera) while
    the MJPEG stream may be configured to another one (640x480, 1280x720).
    Using the calibration fx/cx on a differently sized frame shifts every
    projection laterally *and* in depth, so the matrix is scaled here
    instead of being used as-is.

    Returns ``(K_scaled, sx, sy)``; ``K`` unchanged with ``sx = sy = 1``
    when there is nothing to scale (missing/size-equal input).

    Note: the radtan distortion coefficients are resolution independent
    (normalised), so ``D`` must NOT be rescaled.
    """
    if K is None or not cal_dims or not image_dims:
        return K, 1.0, 1.0
    cal_w, cal_h = float(cal_dims[0]), float(cal_dims[1])
    img_w, img_h = float(image_dims[0]), float(image_dims[1])
    if cal_w <= 0.0 or cal_h <= 0.0 or img_w <= 0.0 or img_h <= 0.0:
        return K, 1.0, 1.0
    sx = img_w / cal_w
    sy = img_h / cal_h
    if sx == 1.0 and sy == 1.0:
        return K, 1.0, 1.0
    scaled = np.array(K, dtype=np.float64, copy=True)
    scaled[0, 0] *= sx
    scaled[1, 1] *= sy
    scaled[0, 2] *= sx
    scaled[1, 2] *= sy
    return scaled, sx, sy


def load_calib_yaml(path):
    """Parse camera calibration yaml.

    Accepts Kalibr cam0 nested format or a flat ROS-ish format.
    Returns dict with: fx, fy, cx, cy, width, height, K(3x3 np),
    D(5 np), distortion_model, camera_model, rostopic.
    """
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    cam = data.get('cam0', data)
    if isinstance(cam, dict) and 'intrinsics' in cam:
        fx, fy, cx, cy = [float(v) for v in cam['intrinsics']]
        res = cam.get('resolution') or [800, 600]
        width, height = int(res[0]), int(res[1])
        dcoeffs = cam.get('distortion_coeffs') or []
        dist_model = cam.get('distortion_model') or 'radtan'
        cam_model = cam.get('camera_model') or 'pinhole'
        rostopic = cam.get('rostopic') or ''
    else:
        fx = float(data.get('fx', data.get('fx', 0.0)))
        fy = float(data.get('fy', 0.0))
        cx = float(data.get('cx', 0.0))
        cy = float(data.get('cy', 0.0))
        width = int(data.get('width', data.get('image_width', 0)))
        height = int(data.get('height', data.get('image_height', 0)))
        dcoeffs = data.get('d', data.get('distortion_coefficients', []))
        dist_model = data.get('distortion_model', '')
        cam_model = data.get('camera_model', 'pinhole')
        rostopic = data.get('rostopic', '')
    d = [float(v) for v in dcoeffs][:5]
    while len(d) < 5:
        d.append(0.0)
    K = np.array([[fx, 0.0, cx],
                  [0.0, fy, cy],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    return {
        'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy,
        'width': width, 'height': height,
        'K': K, 'D': np.array(d, dtype=np.float64),
        'distortion_model': dist_model, 'camera_model': cam_model,
        'rostopic': rostopic,
    }


def undistort_normalized(K, D, u, v):
    """Undistort one pixel -> normalized camera coords (xn, yn)."""
    pts = np.array([[[float(u), float(v)]]], dtype=np.float32)
    out = cv2.undistortPoints(pts, K, D)
    return float(out[0, 0, 0]), float(out[0, 0, 1])


def ground_intersect(origin, direction, z_plane=0.0):
    """Ray-plane intersection with plane z=z_plane.  Returns 3-point or None."""
    dz = float(direction[2])
    if dz >= -1e-9:          # ray points up or parallel -> no hit in front
        return None
    s = (z_plane - float(origin[2])) / dz
    if s < 0.0:
        return None
    pt = np.asarray(origin, dtype=np.float64) + s * np.asarray(direction, dtype=np.float64)
    pt[2] = z_plane
    return pt


def pixel_to_base_point(u, v, K, D, mount, rot, ground_z=0.0, use_distortion=True):
    """Bottom pixel -> ground point in base_footprint.  Returns 3-point or None.

    K/D: camera intrinsics/distortion (D zeroed if use_distortion=False).
    mount: camera optical origin in base (3-vec).
    rot:  R (base <- optical), see rotation_base_optical().
    """
    d = D if use_distortion else np.zeros(5, dtype=np.float64)
    xn, yn = undistort_normalized(K, d, u, v)
    ray_opt = np.array([xn, yn, 1.0], dtype=np.float64)
    origin = np.asarray(mount, dtype=np.float64)
    direction = rot @ ray_opt
    return ground_intersect(origin, direction, ground_z)
