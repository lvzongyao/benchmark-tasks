"""Measure overlap in the sleeve frame and retention in the tray frame."""

from dataclasses import dataclass
from itertools import product
import math

import numpy as np

TRAY_X, TRAY_Y, TRAY_Z = .120, .210, .045
SLEEVE_X, SLEEVE_Y, SLEEVE_Z = .130, .220, .055
BLOCK_X, BLOCK_Y, BLOCK_Z = .060, .100, .030
FLOOR = .004
WALL = .003
SIGNS = np.array(list(product((-1., 1.), repeat=3)))


@dataclass(frozen=True)
class Pose:
    position: np.ndarray
    rotation: np.ndarray

    def local(self, world):
        return (world - self.position) @ self.rotation

    def world(self, local):
        return self.position + local @ self.rotation.T


def corners(x, y, z):
    return SIGNS * np.array([x, y, z]) / 2 + [0., 0., z / 2]


def evaluate(tray, sleeve, block, table, target_x, released):
    """Return the highest current state and its independently measurable witnesses."""
    volume = sleeve.local(tray.world(corners(TRAY_X, TRAY_Y, TRAY_Z)))
    load = tray.local(block.world(corners(BLOCK_X, BLOCK_Y, BLOCK_Z)))
    relative = sleeve.local(tray.position)
    yaw = math.degrees(math.atan2(*(sleeve.rotation.T @ tray.rotation)[[1, 0], 0]))
    mouth = SLEEVE_Y / 2
    gap = relative[1] + TRAY_Y / 2 - mouth
    depth = mouth - relative[1] + TRAY_Y / 2
    retention = (
        np.max(np.abs(load[:, 0])) <= TRAY_X / 2 - WALL + .001
        and np.max(np.abs(load[:, 1])) <= TRAY_Y / 2 - WALL + .001
        and np.min(load[:, 2]) >= FLOOR - .003
        and np.max(load[:, 2]) <= TRAY_Z + .001
        and block.position[2] > table - .01
    )
    # Check the corners of the leading face at entry and the whole tray when seated.
    front = volume[SIGNS[:, 1] < 0]
    entered = (
        np.max(np.abs(front[:, 0])) <= SLEEVE_X / 2 - WALL + .0015
        and np.min(front[:, 2]) >= FLOOR - .003
        and np.max(front[:, 2]) <= SLEEVE_Z - FLOOR + .002
        and np.min(front[:, 1]) >= -.104
        and np.max(front[:, 1]) < mouth
        and abs(yaw) <= 7.
    )
    flush = (
        np.max(np.abs(volume[:, 0])) <= SLEEVE_X / 2 - WALL + .0015
        and np.min(volume[:, 2]) >= FLOOR - .003
        and np.max(volume[:, 2]) <= SLEEVE_Z - FLOOR + .002
        and abs(gap) <= .003 and abs(yaw) <= 4.
    )
    placed = (
        math.hypot(sleeve.position[0] - target_x, sleeve.position[1]) <= .035
        and abs(sleeve.position[2] - table) <= .003
        and sleeve.rotation[2, 2] >= math.cos(math.radians(3.))
        and abs(np.min(load[:, 2]) - FLOOR) <= .003
    )
    info = dict(retained=bool(retention), entered=bool(entered), flush=bool(flush), placed=bool(placed),
                depth_m=float(depth), front_gap_m=float(gap), relative_yaw_deg=yaw,
                load_bottom_m=float(np.min(load[:, 2])))
    if not retention or not entered or min(tray.position[2], sleeve.position[2]) < table - .01:
        return 0., info
    if flush:
        return (1. if placed and released else .75), info
    if depth >= TRAY_Y / 2:
        return .5, info
    return (.25 if depth >= .030 else 0.), info
