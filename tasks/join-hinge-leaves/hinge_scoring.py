"""Geometric pin engagement and a history-based flap rotation witness."""

from dataclasses import dataclass
import math

import numpy as np
from hinge_geometry import PIN_TAIL, PIN_TIP

EAR_OUTSIDE = .0275
EAR_GAP_HALF = .0175
EYE_HALF = .013


@dataclass(frozen=True)
class Engagement:
    entered: bool = False
    through_eye: bool = False
    captured: bool = False


def assess(base_pos, base_rot, flap_pos, flap_rot, pin_pos, pin_rot):
    eye_center = (flap_pos - base_pos) @ base_rot
    pin_center = (pin_pos - base_pos) @ base_rot
    eye_axis = flap_rot[:, 1] @ base_rot
    pin_axis = pin_rot[:, 1] @ base_rot
    if eye_axis[1] < .997 or pin_axis[1] < .997:
        return Engagement()
    if abs(eye_center[1]) + EYE_HALF > EAR_GAP_HALF + .0005:
        return Engagement()
    if np.linalg.norm(eye_center[[0, 2]]) > .003:
        return Engagement()

    def within(y, center, axis, clearance):
        pin_point = pin_center + (y - pin_center[1]) / pin_axis[1] * pin_axis
        bore_point = center + (y - center[1]) / axis[1] * axis
        return np.linalg.norm((pin_point - bore_point)[[0, 2]]) <= clearance

    origin, axial = np.zeros(3), np.array([0., 1., 0.])
    low = pin_center[1] + PIN_TAIL * pin_axis[1]
    high = pin_center[1] + PIN_TIP * pin_axis[1]
    first = low <= -EAR_OUTSIDE + .001 and high >= -EAR_OUTSIDE + .005
    first = first and within(-EAR_OUTSIDE, origin, axial, .002)
    through = first and high >= eye_center[1] + EYE_HALF
    through = through and all(within(y, eye_center, eye_axis, .0016)
                              for y in (eye_center[1] - EYE_HALF, eye_center[1] + EYE_HALF))
    complete = through and high >= EAR_OUTSIDE + .003
    # Check the straight throats. The near bore has 0.8 mm nominal shaft clearance
    # and the far bore has 1.3 mm. Small numerical and contact tolerances apply.
    complete = complete and all(within(y, origin, axial, clearance) for y, clearance in (
        (-.026, .0011), (-.019, .0011), (.019, .0016), (.026, .0016),
    ))
    return Engagement(bool(first), bool(through), bool(complete))


class HingeWitness:
    def __init__(self):
        self.reference = None
        self.verified = False

    def update(self, state, angle):
        if not state.captured:
            self.reference = None
            self.verified = False
        elif self.reference is None:
            self.reference = angle
        else:
            delta = math.remainder(angle - self.reference, 2 * math.pi)
            self.verified |= abs(delta) >= math.radians(15)
        return self.verified


def flap_angle(base_rot, flap_rot):
    relative = base_rot.T @ flap_rot
    return math.atan2(relative[2, 0], relative[0, 0])


def score(engagement, verified, released):
    if engagement.captured:
        return 1. if verified and released else .75
    if engagement.through_eye:
        return .50
    return .25 if engagement.entered else 0.
