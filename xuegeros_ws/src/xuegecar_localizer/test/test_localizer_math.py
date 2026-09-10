#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for the P3 pure geometry helpers (no ROS, no camera).

Regression for the field bug "the bottle is straight ahead but the semantic
map says it is to the side": the calibration yaml is captured at 800x600,
while the MJPEG stream resolution is configured on the camera board and can
be something else.  Feeding the 800x600 matrix to a 640x480 frame shifts
every projection by ~10 degrees.

Written with unittest.TestCase on purpose: the package has no pytest test
type registered, so `colcon test` runs unittest discovery here (pytest
collects these classes as well).
"""

import math
import unittest

import numpy as np

from xuegecar_localizer import localizer_math as lm

#: shipped calibration: OV3660, SVGA 800x600
CAL_K = np.array([[456.2779769888749, 0.0, 403.85938756963463],
                  [0.0, 454.59250678260713, 309.55885730516604],
                  [0.0, 0.0, 1.0]])
CAL_DIMS = (800, 600)
MOUNT = np.array([0.06, 0.0, 0.12])              # optical centre, base_footprint
ROT = lm.rotation_base_optical(0.0, 10.0, 0.0)   # pitched 10 deg down
ZERO_D = np.zeros(5, dtype=np.float64)

#: ground points (base_footprint) used for the round-trip checks
POINTS = [(1.00, 0.00, 0.0), (0.80, 0.15, 0.0), (1.50, -0.20, 0.0),
          (2.00, 0.00, 0.0)]


def project(point):
    """Ground point (base_footprint) -> pixel in the 800x600 frame."""
    p_opt = ROT.T @ (np.asarray(point, dtype=np.float64) - MOUNT)
    assert p_opt[2] > 0.0
    xn, yn = p_opt[0] / p_opt[2], p_opt[1] / p_opt[2]
    return CAL_K[0, 0] * xn + CAL_K[0, 2], CAL_K[1, 1] * yn + CAL_K[1, 2]


class TestScaleIntrinsics(unittest.TestCase):

    def test_identity_when_sizes_match(self):
        K, sx, sy = lm.scale_intrinsics(CAL_K, CAL_DIMS, CAL_DIMS)
        self.assertEqual(sx, 1.0)
        self.assertEqual(sy, 1.0)
        self.assertTrue(np.allclose(K, CAL_K))

    def test_scales_focal_and_principal_point(self):
        K, sx, sy = lm.scale_intrinsics(CAL_K, CAL_DIMS, (640, 480))
        self.assertAlmostEqual(sx, 0.8)
        self.assertAlmostEqual(sy, 0.8)
        self.assertAlmostEqual(K[0, 0], CAL_K[0, 0] * 0.8)
        self.assertAlmostEqual(K[1, 1], CAL_K[1, 1] * 0.8)
        self.assertAlmostEqual(K[0, 2], CAL_K[0, 2] * 0.8)
        self.assertAlmostEqual(K[1, 2], CAL_K[1, 2] * 0.8)

    def test_safe_on_bad_input(self):
        for dims in (None, (0, 0), (800, 0)):
            K, sx, sy = lm.scale_intrinsics(CAL_K, CAL_DIMS, dims)
            self.assertTrue(np.allclose(K, CAL_K))
            self.assertEqual((sx, sy), (1.0, 1.0))
        K, _, _ = lm.scale_intrinsics(None, CAL_DIMS, (640, 480))
        self.assertIsNone(K)


class TestGroundProjection(unittest.TestCase):

    def test_ground_round_trip_at_calibration_resolution(self):
        for point in POINTS:
            u, v = project(point)
            got = lm.pixel_to_base_point(u, v, CAL_K, ZERO_D, MOUNT, ROT,
                                         ground_z=0.0, use_distortion=False)
            self.assertIsNotNone(got)
            for index in range(3):
                # 5 places: the pixel goes through float32 in
                # cv2.undistortPoints, so ~1e-6 is the numerical floor
                self.assertAlmostEqual(got[index], point[index], places=5)

    def test_rescaled_intrinsics_recover_the_same_ground_point(self):
        """The fix: 640x480 stream + rescaled K gives the same answer."""
        K2, sx, sy = lm.scale_intrinsics(CAL_K, CAL_DIMS, (640, 480))
        for point in POINTS:
            u, v = project(point)
            u2, v2 = u * sx, v * sy      # same scene, other resolution
            got = lm.pixel_to_base_point(u2, v2, K2, ZERO_D, MOUNT, ROT,
                                         ground_z=0.0, use_distortion=False)
            self.assertIsNotNone(got)
            for index in range(3):
                self.assertAlmostEqual(got[index], point[index], places=5)

    def test_unscaled_intrinsics_are_off_by_more_than_five_centimetres(self):
        """Without the rescale the target lands far to the side (field bug).

        Depending on where it sits in the frame the un-rescaled ray either
        hits the floor metres away or misses it entirely (None) -- both mean
        the object ends up somewhere other than where it really is.
        """
        for point in POINTS:
            u, v = project(point)
            u2, v2 = u * 0.8, v * 0.6
            wrong = lm.pixel_to_base_point(u2, v2, CAL_K, ZERO_D, MOUNT, ROT,
                                           ground_z=0.0, use_distortion=False)
            if wrong is None:
                continue
            error = math.hypot(wrong[0] - point[0], wrong[1] - point[1])
            self.assertGreater(error, 0.05,
                               'expected a visible error, got %.4f m' % error)

    def test_ray_above_horizon_has_no_ground_hit(self):
        # a pixel above the horizon looks up -> no ground intersection
        self.assertIsNone(
            lm.ground_intersect((0.0, 0.0, 0.12), (1.0, 0.0, 0.5)))


if __name__ == '__main__':
    unittest.main()
