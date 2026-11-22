"""Load a product into a tray with independent loose end protectors."""

import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from packing import (
    CAP_BACK, CAP_CENTER, CAP_DEPTH, CAP_FLOOR, CAP_INNER_X, CAP_X,
    PRODUCT_X, PRODUCT_Y, PRODUCT_Z, SIGNS, TRAY_FLOOR, TRAY_X, TRAY_Y,
    Pose, assess, score as evaluate,
)
from packing_parts import Part, env, on_reset, on_step
from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint, grip_rotation
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

from packing_pose import as_waypoint_rotation

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = .05
INSTRUCTION = "Put both end protectors on the product and load the protected product into the tray."
TOP = TABLE_TOP[SCENE]
WORK_X, TRAY_HOME_X = .245, .392
CAP_HOME_Y = .157
REST_HEIGHT = .015
WALL = .004
TRAY_WALL_Z = .026
GRIP_Z = .073
GRIP_CENTER_X_COMPENSATION = .008
LEFT_COLOR, RIGHT_COLOR = (.95, .48, .12), (.97, .76, .16)
PRODUCT_COLOR, TRAY_COLOR = (.29, .66, .78), (.18, .30, .40)
NAMES = ("product", "left_protector", "right_protector", "packing_tray")


def xyz(values):
    return tuple(float(value) for value in values)


def cuboid(x0, x1, y0, y1, z0, z1, color):
    low, high = np.array((x0, y0, z0)), np.array((x1, y1, z1))
    return xyz((low + high) / 2), xyz(high - low), color


def protector(side, color):
    depth = CAP_DEPTH / 2
    half = CAP_X / 2
    boxes = (
        cuboid(-half, half, -depth, depth, 0., CAP_FLOOR, color),
        cuboid(-half, half, CAP_BACK, depth, CAP_FLOOR, .064, color),
        cuboid(-half, -CAP_INNER_X, -.006, CAP_BACK, CAP_FLOOR, .032, color),
        cuboid(CAP_INNER_X, half, -.006, CAP_BACK, CAP_FLOOR, .032, color),
        cuboid(-.014, .014, -depth, depth, .060, .090, color),
    )
    return tuple(((p[0], side * p[1], p[2]), size, color) for p, size, color in boxes)


def tray():
    x, y = TRAY_X / 2, TRAY_Y / 2
    color = TRAY_COLOR
    return (
        cuboid(-x-WALL, x+WALL, -y-WALL, y+WALL, 0., TRAY_FLOOR, color),
        cuboid(-x-WALL, -x, -y-WALL, y+WALL, TRAY_FLOOR, TRAY_WALL_Z, color),
        cuboid(x, x+WALL, -y-WALL, y+WALL, TRAY_FLOOR, TRAY_WALL_Z, color),
        cuboid(-x, x, -y-WALL, -y, TRAY_FLOOR, TRAY_WALL_Z, color),
        cuboid(-x, x, y, y+WALL, TRAY_FLOOR, TRAY_WALL_Z, color),
    )


product_box = cuboid(
    -PRODUCT_X/2, PRODUCT_X/2, -PRODUCT_Y/2, PRODUCT_Y/2, 0., PRODUCT_Z, PRODUCT_COLOR,
)
PRODUCT = Part(NAMES[0], pos=(WORK_X, 0., TOP + REST_HEIGHT), size=product_box[1],
               mass=.11, boxes=(product_box,), friction=.55)
CAPS = tuple(
    Part(name, pos=(WORK_X, side * CAP_HOME_Y, TOP), size=(CAP_X, CAP_DEPTH, .090),
         mass=.045, boxes=protector(side, color), friction=.7)
    for name, side, color in zip(NAMES[1:3], (1, -1), (LEFT_COLOR, RIGHT_COLOR))
)
TRAY = Part(NAMES[3], pos=(TRAY_HOME_X, 0., TOP), size=(TRAY_X+2*WALL, TRAY_Y+2*WALL, TRAY_WALL_Z),
            mass=.28, boxes=tray(), friction=.7)
OBJECTS = [PRODUCT, *CAPS, TRAY,
           Box("reset_rest", pos=(WORK_X, 0., TOP + REST_HEIGHT/2),
               size=(.058, .092, REST_HEIGHT), color=(.26,.29,.32))]

PARTS = (PRODUCT, *CAPS, TRAY)
HULLS = tuple(np.concatenate([np.array(c) + SIGNS * np.array(s)/2 for c, s, _ in p.boxes])
              for p in PARTS)
DWELL = []
STEP, LAST_STEP, LAST_SCORE = 0, -1, 0.
CONTACTS = None


@on_reset
def reset(environment):
    global STEP, LAST_STEP, LAST_SCORE, CONTACTS
    STEP, LAST_STEP, LAST_SCORE = 0, -1, 0.
    DWELL.clear()
    path = os.environ.get("PACKING_ARM_CONTACTS")
    if path:
        from packing_contacts import ArmContacts
        from isaaclab_physx.physics.physx_manager import PhysxManager

        CONTACTS = ArmContacts(path)
        if not getattr(PhysxManager.step, "_c1_contacts", False):
            native_step = PhysxManager.step.__func__

            def step(cls):
                native_step(cls)
                CONTACTS.poll(cls.get_physics_dt())

            step._c1_contacts = True
            PhysxManager.step = classmethod(step)


@on_step
def tick(environment):
    global STEP
    STEP += 1


def jaws_clear(poses):
    base = env().unwrapped
    clouds = [pose.world(points) for pose, points in zip(poses, HULLS)]
    for side in ("left", "right"):
        arm = base.scene[f"{side}_robot"]
        ids, _ = arm.find_joints([*ARM_JOINTS, *FINGER_JOINTS], preserve_order=True)
        q = arm.data.joint_pos.torch[0, ids].detach().cpu().numpy()
        tcp, rotation = ik.tcp_pose(q[:6], station.BASES[side])
        box = tcp + (SIGNS * [max(abs(q[6:]))+.013, .039, .033] + [0., 0., -.016]) @ rotation.T
        for cloud in clouds:
            separation = np.maximum(cloud.min(0)-box.max(0), box.min(0)-cloud.max(0))
            if separation.max() < .010:
                return False
    return True


def steady(previous, current):
    positions, rotations = current
    old_pos, old_rot = previous
    return (np.all(np.linalg.norm(positions-old_pos, axis=1) <= .002)
            and np.all((np.sum(rotations*old_rot, axis=(1, 2))-1)/2 >= math.cos(math.radians(2))))


def score(objects):
    global LAST_STEP, LAST_SCORE
    if STEP == LAST_STEP:
        return LAST_SCORE
    poses = [Pose(objects[name].pos, objects[name].rotation()) for name in NAMES]
    outcome = assess(poses[0], poses[1:3], poses[3])
    current = np.array([p.position for p in poses]), np.array([p.rotation for p in poses])
    if outcome.loaded and jaws_clear(poses):
        while DWELL and not all(steady(previous, current) for previous in DWELL):
            DWELL.pop(0)
        DWELL.append(current)
        del DWELL[:-20]
    else:
        DWELL.clear()
    LAST_STEP, LAST_SCORE = STEP, evaluate(outcome, len(DWELL) == 20)
    path = os.environ.get("PACKING_DIAGNOSTICS")
    if path:
        row = dict(step=STEP, score=LAST_SCORE, protected=outcome.protected, loaded=outcome.loaded,
                   dwell=len(DWELL), positions=current[0].tolist(), rotations=current[1].tolist(),
                   arm_contacts=CONTACTS.contacts if CONTACTS is not None else None)
        with open(path, "a") as log:
            log.write(json.dumps(row) + "\n")
    return LAST_SCORE


def success(objects):
    return score(objects) == 1.


def demo():
    waypoints = {}
    for arm, side in (("left", 1), ("right", -1)):
        def point(x, y, z):
            return xyz((x, side * (y + .009), TOP + z))
        home = CAP_HOME_Y
        # Hold inward pressure from seating until both shelves rest on the tray floor.
        seated = CAP_CENTER - .002
        work = WORK_X + GRIP_CENTER_X_COMPENSATION
        destination = TRAY_HOME_X + GRIP_CENTER_X_COMPENSATION
        fitted = GRIP_Z + REST_HEIGHT - CAP_FLOOR
        lifted = GRIP_Z + .048
        place = GRIP_Z + TRAY_FLOOR
        steps = (
            (2.4, point(WORK_X, home, .155), FINGER_OPEN, "approach protector"),
            (1.7, point(WORK_X, home, GRIP_Z), FINGER_OPEN, "lower over handle"),
            (.9, point(WORK_X, home, GRIP_Z), FINGER_CLOSED, "grip protector"),
            (1.2, point(work, home, fitted), FINGER_CLOSED, "bring shelf to product height"),
            (3.6, point(work, seated, fitted), FINGER_CLOSED, "fit both end protectors"),
            (1.9, point(work, seated, lifted), FINGER_CLOSED, "lift by the two shelves"),
            (2.6, point(destination, seated, lifted), FINGER_CLOSED, "carry protected product to tray"),
            (2.6, point(destination, seated, place), FINGER_CLOSED, "seat on tray floor"),
            (.9, point(destination, seated, place), FINGER_OPEN, "release onto tray"),
            (2.2, point(destination, seated, .139), FINGER_OPEN, "withdraw both grippers"),
        )
        # Keep the forearms outside the cup pair while the fingers close along x.
        lean = axis_angle_matrix(np.array([1., 0., 0.]), -side * math.radians(14))
        rotation = as_waypoint_rotation(lean @ grip_rotation(-side * math.pi / 2))
        waypoints[arm] = [Waypoint(dt, position, grip, label, rotation) for dt, position, grip, label in steps]
    return waypoints
