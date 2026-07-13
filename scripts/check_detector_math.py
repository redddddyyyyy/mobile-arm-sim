#!/usr/bin/env python3
"""Offline sanity check for block_detector's math. No ROS, no sim.

Two checks:
  color    — synthetic image with the four scene colors; only the red square
             may survive the mask
  geometry — project a known floor point into pixels using the real camera
             intrinsics + mount pose, back-project with pixel_to_ground,
             expect < 1 cm round-trip error

Run: python3 scripts/check_detector_math.py
Exits 1 on the first failure. Cheaper to debug math here than in Gazebo.
"""

import math
import sys

import cv2
import numpy as np

from block_detector import GROUND_Z, largest_blob_centroid, pixel_to_ground, red_mask


def check_color():
    # BGR values picked to land at the same hues as the spawned blocks:
    # orange ~13, magenta ~150, brown ~10 (and darker). Only red may pass.
    img = np.full((480, 640, 3), 120, np.uint8)
    squares = [
        ('red',     (0, 0, 200),   60),
        ('orange',  (0, 90, 200),  200),
        ('magenta', (200, 0, 200), 340),
        ('brown',   (20, 50, 110), 480),
    ]
    y = 210
    for _, bgr, x in squares:
        img[y:y + 60, x:x + 60] = bgr

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = red_mask(hsv)

    hit = largest_blob_centroid(mask)
    if hit is None:
        print('FAIL color: red square not detected at all')
        return False
    u, v, area = hit
    if not (60 <= u <= 120 and y <= v <= y + 60):
        print(f'FAIL color: centroid ({u:.0f}, {v:.0f}) outside the red square')
        return False

    # Nothing may fire outside the red square (small margin for the
    # morphological close bleeding the edge by a pixel or two).
    outside = mask.copy()
    outside[y - 5:y + 65, 55:125] = 0
    if cv2.countNonZero(outside):
        print(f'FAIL color: {cv2.countNonZero(outside)} mask pixels outside the red square')
        return False

    print('PASS color')
    return True


def check_geometry():
    # Real intrinsics: 640x480, HFOV 1.2 rad -> fx = 320 / tan(0.6)
    fx = 320.0 / math.tan(0.6)
    K = np.array([[fx, 0.0, 320.0],
                  [0.0, fx, 240.0],
                  [0.0, 0.0, 1.0]])

    # Real mount, straight from the URDF: base_link sits wheel_radius = 0.05 m
    # above the floor, camera_joint is xyz=(0.20, 0, 0.18) pitched 0.349 rad
    # down. Robot assumed at map origin with zero yaw.
    pitch = 0.349
    t_mo = np.array([0.20, 0.0, 0.05 + 0.18])
    c, s = math.cos(pitch), math.sin(pitch)
    R_base_cam = np.array([[c, 0.0, s],
                           [0.0, 1.0, 0.0],
                           [-s, 0.0, c]])
    # camera_link -> camera_link_optical, the fixed rpy=(-pi/2, 0, -pi/2)
    # joint: optical z forward, x right, y down.
    R_cam_opt = np.array([[0.0, 0.0, 1.0],
                          [-1.0, 0.0, 0.0],
                          [0.0, -1.0, 0.0]])
    R_mo = R_base_cam @ R_cam_opt

    # The optical axis meets the floor ~0.63 m ahead of the lens, so this
    # point lands comfortably inside the frame.
    p_true = np.array([0.90, 0.05, GROUND_Z])

    # Forward-project with independent math (don't reuse pixel_to_ground).
    p_opt = R_mo.T @ (p_true - t_mo)
    if p_opt[2] <= 0:
        print('FAIL geometry: test point behind the camera — extrinsics wrong')
        return False
    u = fx * p_opt[0] / p_opt[2] + 320.0
    v = fx * p_opt[1] / p_opt[2] + 240.0
    if not (0 <= u < 640 and 0 <= v < 480):
        print(f'FAIL geometry: test point projects off-frame at ({u:.1f}, {v:.1f})')
        return False

    p_back = pixel_to_ground(u, v, K, R_mo, t_mo)
    if p_back is None:
        print('FAIL geometry: pixel_to_ground rejected a valid ground pixel')
        return False
    err = float(np.linalg.norm(p_back - p_true))
    if err >= 0.01:
        print(f'FAIL geometry: round-trip error {err * 100:.2f} cm (limit 1 cm)')
        return False

    print(f'PASS geometry (round-trip {err * 1000:.3f} mm at pixel ({u:.0f}, {v:.0f}))')
    return True


if __name__ == '__main__':
    ok = check_color() and check_geometry()
    sys.exit(0 if ok else 1)
