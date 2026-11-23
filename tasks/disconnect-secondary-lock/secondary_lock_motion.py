"""Give the expert stock YAM waypoints an optional outward wrist tilt."""

import math

import numpy as np
from yam_arena import demo
from yam_arena.urdf import axis_angle_matrix


class TiltedYaw(float):
    def __new__(cls, yaw, tilt):
        value = super().__new__(cls, yaw)
        value.tilt = tilt
        return value


_vertical_rotation = demo.grip_rotation


def grip_rotation(yaw):
    rotation = _vertical_rotation(yaw)
    if isinstance(yaw, TiltedYaw):
        return axis_angle_matrix(np.array([1., 0., 0.]), yaw.tilt) @ rotation
    return rotation


demo.grip_rotation = grip_rotation
LEFT_YAW = TiltedYaw(-math.pi / 2, math.radians(-15))
RIGHT_YAW = TiltedYaw(math.pi / 2, math.radians(15))
