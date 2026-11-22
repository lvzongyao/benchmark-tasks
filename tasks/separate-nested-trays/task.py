"""Separate two reusable trays by disengaging their retaining lips, then stage them upright."""

from dataclasses import asdict
from pathlib import Path
import math
import os
import sys
import json

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from partkit import env, on_reset, on_step, part
from scoring import Dwell, Inputs, overlap_depth, score as evaluate
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import Box, TABLE_TOP

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = .05  # The scorer itself checks twenty distinct control steps.
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Separate the nested trays and place the orange tray upright on the left pad and the blue tray upright on the right pad."
CENTER_X = .265
PAD_Y = .211
PAD_X = .265
PAD_H = .003
PAD_SIZE = np.array([.254, .203])
RELEASE = .008
BLUE = (.10, .36, .82)
ORANGE = (.99, .45, .09)
PAD_COLOR = (.20, .56, .31)
WITNESS = (.94, .88, .18)


def box_mm(x0, x1, y0, y1, z0, z1, color):
    lo, hi = np.array([x0, y0, z0]), np.array([x1, y1, z1])
    return tuple((lo + hi) / 2000), tuple((hi - lo) / 1000), color


def shell(length, width, height, color):
    x, y = length / 2, width / 2
    return [box_mm(-x, x, -y, y, 0, 3, color),
            box_mm(-x, x, -y, -y + 3, 3, height, color),
            box_mm(-x, x, y - 3, y, 3, height, color),
            box_mm(-x, -x + 3, -y + 3, y - 3, 3, height, color),
            box_mm(x - 3, x, -y + 3, y - 3, 3, height, color)]


# Both 4 mm shoulders overlap at reset. An 8 mm shift leaves clearance for small yaw error.
OUTER_BOXES = [*shell(220, 150, 45, BLUE),
               box_mm(-2, 2, -72, -63, 39, 44, WITNESS),
               box_mm(-2, 2, 63, 72, 39, 44, WITNESS),
               box_mm(-12, 12, -90, -74, 26, 73, BLUE)]
INNER_BOXES = [*shell(180, 120, 35, ORANGE),
               box_mm(-2, 2, -70, -59, 26, 30, ORANGE),
               box_mm(-2, 2, 59, 70, 26, 30, ORANGE),
               box_mm(-12, 12, 49, 60, 26, 68, ORANGE),
               box_mm(-12, 12, 6, 57, 54, 68, ORANGE)]
OUTER = part("blue_tray", (CENTER_X, 0., TOP + .00015), OUTER_BOXES, .24, friction=.42)
INNER = part("orange_tray", (CENTER_X, 0., TOP + .0033), INNER_BOXES, .17, friction=.42)
OBJECTS = [
    OUTER, INNER,
    Box("left_pad", pos=(PAD_X, PAD_Y, TOP + PAD_H / 2), size=(*PAD_SIZE, PAD_H), color=PAD_COLOR),
    Box("right_pad", pos=(PAD_X, -PAD_Y, TOP + PAD_H / 2), size=(*PAD_SIZE, PAD_H), color=PAD_COLOR),
]

SIGNS = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])


def corners(parts):
    return np.concatenate([np.array(c) + SIGNS * np.array(s) / 2 for c, s, _ in parts])


HULLS = (corners(OUTER_BOXES), corners(INNER_BOXES))
TONGUES = corners(INNER_BOXES[5:7]).reshape(2, 8, 3)
ROOFS = corners(OUTER_BOXES[5:7]).reshape(2, 8, 3)
DWELL = Dwell()
STEP = 0
DIAGNOSTICS = {}
ORIGIN = None


@on_reset
def reset(environment=None):
    global STEP, DWELL, ORIGIN
    STEP, DWELL, ORIGIN = 0, Dwell(), None
    DIAGNOSTICS.clear()


@on_step
def tick(environment):
    global STEP
    STEP += 1


def clearance(low, high, other_low, other_high):
    """Distance between axis-aligned enclosing boxes (conservative clearance)."""
    return float(np.linalg.norm(np.maximum(0., np.maximum(low - other_high, other_low - high))))


def hand_boxes():
    """World AABBs conservatively enclosing both jaws around each measured TCP."""
    from yam_arena import ik, station

    base = env().unwrapped
    for name in ("left", "right"):
        robot = base.scene[f"{name}_robot"]
        ids, _ = robot.find_joints([*ARM_JOINTS, *FINGER_JOINTS], preserve_order=True)
        q = robot.data.joint_pos.torch[0, ids].detach().cpu().numpy()
        tcp, rot = ik.tcp_pose(q[:6], station.BASES[name])
        opening = max(abs(q[6:]))
        jaw_box = tcp + (SIGNS * [opening + .014, .039, .036] + [0., 0., -.020]) @ rot.T
        yield jaw_box.min(0), jaw_box.max(0)


def measure(objects, hands=None):
    outer, inner = (objects[name] for name in ("blue_tray", "orange_tray"))
    poses = (outer.pos.copy(), outer.rotation(), inner.pos.copy(), inner.rotation())
    if not all(np.isfinite(p).all() for p in poses):
        return Inputs(invalid=True), {}
    world = [points @ rot.T + pos for points, pos, rot in zip(HULLS, poses[::2], poses[1::2])]
    bounds = [(points.min(0), points.max(0)) for points in world]
    gap = clearance(*bounds[0], *bounds[1])
    # Assess overlap in the lower tray frame, including relative yaw and pitch.
    local_tabs = ((TONGUES @ poses[3].T + poses[2]) - poses[0]) @ poses[1]
    slide_clear = all(
        tab[:, 0].min() >= roof[:, 0].max() + .0005
        or tab[:, 0].max() <= roof[:, 0].min() - .0005
        for tab, roof in zip(local_tabs, ROOFS)
    )
    upright = all(rot[2, 2] >= math.cos(math.radians(10)) for rot in poses[1::2])
    on_pads = []
    for i, (lo, hi) in enumerate(bounds):
        pad = np.array([PAD_X, (-1 if i == 0 else 1) * PAD_Y])
        on_pads.append(
            bool(np.all(lo[:2] >= pad - PAD_SIZE / 2) and np.all(hi[:2] <= pad + PAD_SIZE / 2)
                 and abs(lo[2] - (TOP + PAD_H)) <= .002)
        )
    hands = hand_boxes() if hands is None else hands
    hand_gap = min(clearance(*tray, *hand) for hand in hands for tray in bounds)
    relative_rotation = poses[1].T @ poses[3]
    lip_overlap = max(overlap_depth(tab, roof, relative_rotation) for tab, roof in zip(local_tabs, ROOFS))
    forced_penetration = lip_overlap > .0025
    invalid = forced_penetration or any(lo[2] < TOP - .05 for lo, _ in bounds)
    inputs = Inputs(invalid=invalid, unlocked=slide_clear, separate=gap >= .002,
                    upright=upright, staged=all(on_pads) and gap >= .030, clear=hand_gap >= .010, poses=poses)
    return inputs, {
        "lip_overlap_mm": lip_overlap * 1000, "forced_penetration": forced_penetration,
        "gap_mm": gap * 1000, "hand_clearance_mm": hand_gap * 1000, "on_pads": on_pads,
        "relative_x_mm": float(((poses[2] - poses[0]) @ poses[1])[0] * 1000),
        "relative_yaw_deg": math.degrees(math.atan2(relative_rotation[1, 0], relative_rotation[0, 0])),
        "tab_x_ranges_mm": [[float(tab[:, 0].min() * 1000), float(tab[:, 0].max() * 1000)] for tab in local_tabs],
        "outer_xyz": poses[0].tolist(), "inner_xyz": poses[2].tolist(),
    }


def score(objects):
    if DWELL.step == STEP:
        return DWELL.result
    inputs, details = measure(objects)
    result = evaluate(inputs, DWELL, STEP)
    flags = {k: bool(v) for k, v in vars(inputs).items() if k != "poses"}
    DIAGNOSTICS.update(details, **flags, step=STEP, dwell=len(DWELL.history),
                       invalid_latched=bool(DWELL.invalid), score=result)
    target = os.environ.get("TRAY_DIAGNOSTICS")
    if target:
        with open(target, "a") as log:
            log.write(json.dumps(DIAGNOSTICS) + "\n")
    return result


def success(objects):
    return score(objects) == 1.


def demo():
    """Cartesian waypoints executed through the stock scripted policy and joint targets."""
    mode = os.environ.get("TRAY_DEMO", "full")
    # Pinch the flat sides of each raised grasp land with mirrored wrist rolls.
    left_yaw, right_yaw = -math.pi / 2, math.pi / 2
    opened, held = FINGER_OPEN, FINGER_CLOSED
    hover = TOP + .176
    left_grasp, right_grasp = TOP + .063, TOP + .064
    ix, iy = CENTER_X, .020
    ox, oy = CENTER_X, -.082
    left, right = [], []

    def both(duration, lpos, lgrip, rpos, rgrip, label, left_turn=0., right_turn=0.):
        left.append(Waypoint(duration, lpos, lgrip, label, left_yaw + left_turn))
        right.append(Waypoint(duration, rpos, rgrip, label, right_yaw + right_turn))

    upper, lower = (ix, iy, left_grasp), (ox, oy, right_grasp)
    both(2.0, (ix, iy, hover), opened, (ox, oy, hover), opened, "approach")
    both(2.0, upper, opened, lower, opened, "descend")
    both(1.0, upper, held, lower, held, "grasp")
    if mode == "straight":
        axial_pull = (ix, iy, left_grasp + .014)
        both(2.0, axial_pull, held, lower, held, "short straight pull")
        both(1.0, axial_pull, held, lower, held, "hold pull")
        return {"left": left, "right": right}
    if mode == "together":
        both(2.5, (ix, iy, left_grasp + .014), held, (ox, oy, right_grasp + .014), held, "lift engaged pair")
        return {"left": left, "right": right}
    # Command travel beyond the nominal shift to account for pad compliance and counterhold drift.
    sx = ix + 2 * RELEASE
    counterhold = (ox - .012, oy, right_grasp)
    slide_z = left_grasp + .003
    both(.6, (ix, iy, slide_z), held, lower, held, "unload base contact within the lips")
    correction = 0.
    both(2.0, (sx, iy, slide_z), held, counterhold, held, "slide and square the inner tray", correction)
    if mode == "slide":
        both(.8, (sx, iy, left_grasp), opened, counterhold, opened, "release after slide", correction)
        both(2., (sx, iy, hover), opened, (counterhold[0], oy, hover), opened, "retreat")
        return {"left": left, "right": right}
    both(2.0, (sx, iy, hover), held, counterhold, held, "lift inner tray", correction)
    lp, rp = (PAD_X, PAD_Y + iy + .033), (PAD_X, -PAD_Y + oy)
    both(2.0, (*lp, hover), held, counterhold, held, "carry inner tray")
    both(2.0, (*lp, left_grasp + .003), held, counterhold, held, "lower inner tray")
    both(.8, (*lp, left_grasp + .003), opened, counterhold, held, "release inner tray")
    lower_pick = (.201, -.067, right_grasp)
    lower_turn = math.radians(-10)
    both(.6, (*lp, left_grasp + .003), opened, counterhold, opened, "open right jaws")
    both(1.4, (*lp, hover), opened, (*counterhold[:2], hover), opened, "clear the trays")
    if mode == "one_pad":
        return {"left": left, "right": right}
    both(1.0, (*lp, hover), opened, (*lower_pick[:2], hover), opened, "align right grasp to outer tray", right_turn=lower_turn)
    both(1.5, (*lp, hover), opened, lower_pick, opened, "descend over outer grip", right_turn=lower_turn)
    both(1.0, (*lp, hover), opened, lower_pick, held, "regrasp outer tray", right_turn=lower_turn)
    both(2.0, (*lp, hover), opened, (*lower_pick[:2], hover), held, "lift outer tray", right_turn=lower_turn)
    both(2.0, (*lp, hover), opened, (*rp, hover), held, "carry outer tray")
    both(2.0, (*lp, hover), opened, (*rp, right_grasp + PAD_H), held, "lower outer tray")
    both(.8, (*lp, hover), opened, (*rp, right_grasp + PAD_H), opened, "release outer tray")
    both(2.0, (*lp, hover), opened, (*rp, hover), opened, "retreat")
    return {"left": left, "right": right}
