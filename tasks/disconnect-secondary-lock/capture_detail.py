"""Capture the two mechanical locks and seating shoulder from an oblique camera."""

from dataclasses import replace

import numpy as np
from inspect_robots.cli import main
from yam_arena import ik, inspect_embodiment, station


def use_detail_camera():
    eye = np.array([.500, -.267, .998])
    aim = np.array([.310, -.032, .800])
    back = eye - aim
    back /= np.linalg.norm(back)
    right = np.cross([0., 0., 1.], back)
    right /= np.linalg.norm(right)
    up = np.cross(back, right)
    rotation = ik.matrix_to_quat_xyzw(np.column_stack([right, up, back]))
    stock_rig = station.rig
    stock_stand = station.camera_stand

    def rig(variant="calibrated"):
        calibrated = stock_rig(variant)
        return replace(calibrated, top_cam=replace(calibrated.top_cam, pos=tuple(eye),
                                                    rot_xyzw=tuple(rotation)))

    station.rig = rig
    station.camera_stand = lambda camera: stock_stand(station.top_camera())


if __name__ == "__main__":
    inspect_embodiment.RESET_RENDERS = 96
    use_detail_camera()
    raise SystemExit(main())
