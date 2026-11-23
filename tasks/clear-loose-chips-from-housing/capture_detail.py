"""Record the chute and receiving cup from an oblique angle."""

from dataclasses import replace

import numpy as np
from inspect_robots.cli import main
from yam_arena import ik, inspect_embodiment, station


def detail_camera(camera):
    position = np.array([.485,-.116,.982])
    target = np.array([.267,.106,.793])
    back = position - target
    back /= np.linalg.norm(back)
    right = np.cross([0., 0., 1.], back)
    right /= np.linalg.norm(right)
    up = np.cross(back, right)
    rotation = ik.matrix_to_quat_xyzw(np.column_stack([right, up, back]))
    return replace(camera, pos=tuple(position), rot_xyzw=tuple(rotation))


if __name__ == "__main__":
    stock_rig = station.rig
    stock_stand = station.camera_stand

    def rig(variant="calibrated"):
        calibrated = stock_rig(variant)
        return replace(calibrated, top_cam=detail_camera(calibrated.top_cam))

    station.rig = rig
    station.camera_stand = lambda camera: stock_stand(station.top_camera())
    inspect_embodiment.RESET_RENDERS = 96
    raise SystemExit(main())
