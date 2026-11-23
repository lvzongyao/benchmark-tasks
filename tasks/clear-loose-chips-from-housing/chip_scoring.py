"""Current chip containment in the cup's reference frame."""

import numpy as np

CHIP_SIZE = (.006,.006,.004)
CUP_X, CUP_Y = .120, .100
CUP_FLOOR, CUP_RIM = .004, .035
SIGNS = np.array([(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)])
CHIP_CORNERS = SIGNS * np.asarray(CHIP_SIZE)/2


def captured(local_chip):
    cloud = np.atleast_2d(local_chip)
    lo,hi = cloud.min(0),cloud.max(0)
    return bool(
        np.max(np.abs(cloud[:,0])) <= CUP_X/2+.001
        and np.max(np.abs(cloud[:,1])) <= CUP_Y/2+.001
        and CUP_FLOOR-.0015 <= lo[2] <= CUP_FLOOR+.014
        and hi[2] <= CUP_RIM
    )


def count_captured(chips):
    return sum(captured(chip) for chip in chips)


def score(count,valid,released):
    if not valid:
        return 0.
    if count == 10 and released:
        return 1.
    if count >= 8:
        return .75
    if count >= 5:
        return .50
    return .25 if count >= 3 else 0.
