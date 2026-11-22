"""Dimensions and geometric scoring for two independent loose end protectors."""

from dataclasses import dataclass
import math

import numpy as np

PRODUCT_X = .070
PRODUCT_Y = .150
PRODUCT_Z = .040
CAP_X = .082
CAP_DEPTH = .025
CAP_CENTER = .0665
CAP_INNER_X = .0395
CAP_FLOOR = .004
CAP_BACK = .0085
TRAY_X = .086
TRAY_Y = .162
TRAY_FLOOR = .004
SIGNS = np.array([(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])


def corners(width, length, height):
    return (SIGNS * [width, length, height] + [0., 0., height]) / 2


PRODUCT = corners(PRODUCT_X, PRODUCT_Y, PRODUCT_Z)
CAP_SOLE = corners(CAP_X, CAP_DEPTH, 0.)[::2]


@dataclass(frozen=True)
class Pose:
    position: np.ndarray
    rotation: np.ndarray

    def world(self, points):
        return points @ self.rotation.T + self.position

    def local(self, points):
        return (points - self.position) @ self.rotation


@dataclass(frozen=True)
class Packing:
    protected: int = 0
    loaded: bool = False


def _fits(cap, end, side):
    end = cap.local(end)
    axial = end[:, 1] * side
    bottom = end[::2, 2]
    return (
        np.abs(end[:, 0]).max() <= CAP_INNER_X + .001
        and axial.min() + CAP_DEPTH / 2 >= .015
        and axial.max() <= CAP_BACK + .0015
        and np.all(np.abs(bottom - CAP_FLOOR) <= .003)
        and abs(bottom[0] - bottom[1]) <= .003
    )


def assess(product, caps, tray):
    poses = (product, *caps, tray)
    if not all(np.isfinite(pose.position).all() and np.isfinite(pose.rotation).all() for pose in poses):
        return Packing()

    vertices = product.world(PRODUCT)
    protected = sum(
        bool(_fits(cap, vertices[SIGNS[:, 1] == side], side))
        for side, cap in zip((1, -1), caps)
    )
    if protected != 2:
        return Packing(protected)

    product_in_tray = tray.local(vertices)
    frame = tray.rotation.T @ product.rotation
    yaw = math.atan2(frame[1, 0], frame[0, 0])
    if abs(yaw) > math.radians(5) or frame[2, 2] < math.cos(math.radians(6)):
        return Packing(protected)
    if np.any(np.abs(product_in_tray[:, :2]) > [TRAY_X / 2, TRAY_Y / 2]):
        return Packing(protected)

    for side, cap in zip((1, -1), caps):
        sole = tray.local(cap.world(CAP_SOLE))
        if np.any(np.abs(sole[:, :2]) > [TRAY_X / 2 + .0005, TRAY_Y / 2 + .0005]):
            return Packing(protected)
        if np.any(np.abs(sole[:, 2] - TRAY_FLOOR) > .002):
            return Packing(protected)
        if abs(TRAY_Y / 2 - np.max(side * sole[:, 1])) > .003:
            return Packing(protected)
    return Packing(protected, True)


def score(packing, released):
    if packing.loaded:
        return 1. if released else .75
    return .25 * packing.protected
