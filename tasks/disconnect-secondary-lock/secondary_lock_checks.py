"""Score ordered lock clearance, withdrawal and stable staging."""

import numpy as np

ARM_NAMES = ("left", "right")
SIGNS = np.array([(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])


def vertices(part):
    points = [np.array(p) + SIGNS * np.array(s) / 2 for p, s, *_ in part.boxes]
    points.extend(np.array(piece) for piece, _ in part.wedges)
    return np.concatenate(points)


def world(object_state, points):
    return points @ object_state.rotation().T + object_state.pos


def clear(hand_bounds, parts):
    for low, high in hand_bounds:
        for points in parts:
            gap = np.maximum(points.min(0) - high, low - points.max(0)).max()
            if gap < .010:
                return False
    return True


class Dwell:
    def __init__(self):
        self.poses = []

    def update(self, eligible, states):
        if not eligible:
            self.poses.clear()
            return 0
        poses = np.array([part.pos for part in states])
        rotations = np.array([part.rotation() for part in states])
        while self.poses:
            compatible = all(np.all(np.linalg.norm(poses - p, axis=1) <= .002)
                             and np.all((np.sum(rotations * r, axis=(1, 2)) - 1) / 2 >= .999390827)
                             for p, r in self.poses)
            if compatible:
                break
            self.poses.pop(0)
        self.poses.append((poses, rotations))
        self.poses = self.poses[-20:]
        return len(self.poses)

class Unlock:
    """Track clearance while the tooth passes the shoulder."""
    def __init__(self):
        self.clear_before = False
        self.escaped = False
        self.invalid = False

    def update(self, travel, depression, withdrawal, aligned):
        slider_open = travel >= .010
        tooth_clear = slider_open and depression >= .004
        if withdrawal <= .001:
            self.clear_before = False
            self.escaped = False
        if aligned and .001 < withdrawal <= .012:
            self.clear_before = tooth_clear
        if aligned and withdrawal >= .012 and not self.escaped:
            if self.clear_before and tooth_clear:
                self.escaped = True
            else:
                self.invalid = True
        if withdrawal >= .020 and not self.escaped:
            self.invalid = True
        return slider_open, tooth_clear


def evaluate(slider_open, tooth_clear, withdrawal, escaped, separated, staged, dwell, invalid):
    if invalid:
        return 0.
    if escaped and separated and staged and dwell >= 20:
        return 1.
    if escaped and withdrawal >= .015:
        return .75
    if tooth_clear:
        return .50
    return .25 if slider_open else 0.
