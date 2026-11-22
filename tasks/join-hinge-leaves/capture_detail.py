"""Run the Inspect Robots CLI with a far-ear view of the pivot pin and flap."""

from dataclasses import replace

import numpy as np
from inspect_robots.cli import main
from yam_arena import ik, station


def detail_camera(camera):
    position = np.array([.448, .182, .955])
    target = np.array([.304, -.054, .818])
    back = position - target
    back /= np.linalg.norm(back)
    right = np.cross([0., 0., 1.], back)
    right /= np.linalg.norm(right)
    up = np.cross(back, right)
    rotation = ik.matrix_to_quat_xyzw(np.column_stack([right, up, back]))
    return replace(camera, pos=tuple(position), rot_xyzw=tuple(rotation))


def use_detail_camera():
    stock_rig = station.rig
    stock_stand = station.camera_stand

    def rig(variant="calibrated"):
        calibrated = stock_rig(variant)
        return replace(calibrated, top_cam=detail_camera(calibrated.top_cam))

    station.rig = rig
    # The review camera has no collision geometry and leaves the station's stand at its normal pose.
    station.camera_stand = lambda camera: stock_stand(station.top_camera())


if __name__ == "__main__":
    use_detail_camera()
    raise SystemExit(main())
