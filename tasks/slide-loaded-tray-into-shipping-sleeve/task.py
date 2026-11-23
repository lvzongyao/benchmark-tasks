"""Counterhold a free shipping sleeve while inserting a loaded tray."""

import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_contacts import ArmContacts
from rigid_parts import Part, env, on_reset, on_step
from scoring import (
    BLOCK_X, BLOCK_Y, BLOCK_Z, FLOOR, SIGNS, SLEEVE_X, SLEEVE_Y, SLEEVE_Z,
    TRAY_X, TRAY_Y, TRAY_Z, WALL, Pose, evaluate,
)
from wrist_pose import as_waypoint_rotation
from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint, grip_rotation
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = .05
INSTRUCTION = (
    "Slide the loaded tray all the way into its outer sleeve without spilling the product. "
    "Set the closed package down between the arms, about 34 cm forward from the line through their bases, and let go."
)
TOP = TABLE_TOP[SCENE]
HOME_X, INSERT_X, WORK_X = .210, .246, .340
TRAY_HOME_Y, SLEEVE_HOME_Y = .181, -.135
INSERT_Y = -.103
TRAY_HANDLE_Y, SLEEVE_HANDLE_Y = .130, -.137
TRAY_GRASP_Z, SLEEVE_GRASP_Z = .068, .071
TRAY_COLOR, HANDLE_COLOR = (.88, .43, .15), (.95, .62, .22)
SLEEVE_COLOR, SLEEVE_GRIP = (.23, .35, .48), (.43, .61, .72)
PRODUCT_COLOR = (.54, .76, .35)
GLAZING_COLOR = (.84, .91, .93)
NAMES = ("loaded_tray", "shipping_sleeve", "product")


def xyz(values):
    return tuple(float(value) for value in values)


def cuboid(x0, x1, y0, y1, z0, z1, color):
    low, high = np.array((x0, y0, z0)), np.array((x1, y1, z1))
    return xyz((low + high) / 2), xyz(high - low), color


def tray():
    x, y = TRAY_X / 2, TRAY_Y / 2
    return (
        cuboid(-x, x, -y, y, 0., FLOOR, TRAY_COLOR),
        cuboid(-x, -x+WALL, -y, y, FLOOR, TRAY_Z, TRAY_COLOR),
        cuboid(x-WALL, x, -y, y, FLOOR, TRAY_Z, TRAY_COLOR),
        cuboid(-x+WALL, x-WALL, -y, -y+WALL, FLOOR, TRAY_Z, TRAY_COLOR),
        cuboid(-x+WALL, x-WALL, y-WALL, y, FLOOR, TRAY_Z, TRAY_COLOR),
        cuboid(-.020, .020, y, .150, .027, .043, HANDLE_COLOR),
        cuboid(-.017, .017, .108, .151, .043, .081, HANDLE_COLOR),
    )


def sleeve():
    x, y = SLEEVE_X / 2, SLEEVE_Y / 2
    return (
        cuboid(-x, x, -y, .100, 0., FLOOR, SLEEVE_COLOR),
        cuboid(-x, -x+WALL, -y, .096, FLOOR, SLEEVE_Z, SLEEVE_COLOR),
        cuboid(x-WALL, x, -y, .096, FLOOR, SLEEVE_Z, SLEEVE_COLOR),
        cuboid(-x+WALL, x-WALL, -y, -.100, FLOOR, SLEEVE_Z-FLOOR, SLEEVE_COLOR),
        cuboid(-x+WALL, x-WALL, -y, -.071, SLEEVE_Z-FLOOR, SLEEVE_Z, SLEEVE_COLOR),
        cuboid(-x+WALL, x-WALL, .084, y, SLEEVE_Z-FLOOR, SLEEVE_Z, SLEEVE_COLOR),
        cuboid(-x+WALL, -.043, -.071, .084, SLEEVE_Z-FLOOR, SLEEVE_Z, SLEEVE_COLOR),
        cuboid(.043, x-WALL, -.071, .084, SLEEVE_Z-FLOOR, SLEEVE_Z, SLEEVE_COLOR),
        cuboid(-.020, .020, -.153, -y, .029, .046, SLEEVE_GRIP),
        cuboid(-.017, .017, -.157, -.116, .046, .085, SLEEVE_GRIP),
    )


def entry_chamfers():
    edge, mouth = .096, SLEEVE_Y/2
    half = SLEEVE_X/2
    ramps = [[
        (-half, .100, 0.), (half, .100, 0.), (half, mouth, 0.), (-half, mouth, 0.),
        (-half, .100, FLOOR), (half, .100, FLOOR),
        (half, mouth, .0005), (-half, mouth, .0005),
    ]]
    for side in (-1, 1):
        poly = [(half-WALL, edge), (half, edge), (half, mouth), (half-.0008, mouth)]
        if side < 0:
            poly = [(-x, y) for x, y in reversed(poly)]
        ramps.append([(x, y, z) for z in (0., SLEEVE_Z) for x, y in poly])
    return tuple((tuple(vertices), SLEEVE_COLOR) for vertices in ramps)


WINDOW = cuboid(-.043, .043, -.071, .084, SLEEVE_Z-FLOOR, SLEEVE_Z, GLAZING_COLOR)
product_box = cuboid(-BLOCK_X/2, BLOCK_X/2, -BLOCK_Y/2, BLOCK_Y/2, 0., BLOCK_Z, PRODUCT_COLOR)
TRAY = Part(NAMES[0], pos=(HOME_X, TRAY_HOME_Y, TOP), size=(TRAY_X, TRAY_Y, TRAY_Z),
            mass=.10, boxes=tray(), friction=.36)
SLEEVE = Part(NAMES[1], pos=(HOME_X, SLEEVE_HOME_Y, TOP), size=(SLEEVE_X, SLEEVE_Y, SLEEVE_Z),
              mass=.15, boxes=sleeve(), wedges=entry_chamfers(), optical_panels=(WINDOW,), friction=.5)
PRODUCT = Part(NAMES[2], pos=(HOME_X, TRAY_HOME_Y, TOP+FLOOR), size=product_box[1],
               mass=.065, boxes=(product_box,), friction=.55)
OBJECTS = [TRAY, SLEEVE, PRODUCT]
HULLS = tuple(np.concatenate([np.array(c) + SIGNS * np.array(s)/2 for c, s, _ in (*p.boxes, *p.optical_panels)])
              for p in OBJECTS)

DWELL = []
STEP, LAST_STEP, LAST_SCORE = 0, -1, 0.
CONTACTS = None
GRASP_VIEW = None
GRASP_COUNTS = np.zeros((4, 2), dtype=int)


@on_reset
def reset(environment):
    global STEP, LAST_STEP, LAST_SCORE, CONTACTS, GRASP_VIEW
    STEP, LAST_STEP, LAST_SCORE = 0, -1, 0.
    DWELL.clear()
    GRASP_COUNTS.fill(0)
    path = os.environ.get("SLEEVE_ARM_CONTACTS")
    if path:
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics.physx_manager import PhysxManager

        CONTACTS = ArmContacts(path)
        fingers = [p for p in CONTACTS.left+CONTACTS.right if p.rsplit("/",1)[-1] in ("tip_left","tip_right")]
        paths = [prim.GetPath().pathString for prim in get_current_stage().Traverse()]
        bodies = [next(p for p in paths if p.lower().endswith(f"/{name}/body")) for name in NAMES[:2]]
        GRASP_VIEW = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
            fingers, filter_patterns=[bodies]*4, max_contact_data_count=128)
        if not getattr(PhysxManager.step, "_c2_contacts", False):
            native_step = PhysxManager.step.__func__

            def step(cls):
                native_step(cls)
                dt = cls.get_physics_dt()
                CONTACTS.poll(dt)
                forces = np.asarray(GRASP_VIEW.get_contact_force_matrix(dt=dt).numpy())
                GRASP_COUNTS[:] += np.linalg.norm(forces, axis=-1) > .10

            step._c2_contacts = True
            PhysxManager.step = classmethod(step)


@on_step
def tick(environment):
    global STEP
    STEP += 1


def grip_state():
    base = env().unwrapped
    result = []
    for side in ("left", "right"):
        arm = base.scene[f"{side}_robot"]
        ids, _ = arm.find_joints([*ARM_JOINTS, *FINGER_JOINTS], preserve_order=True)
        q = arm.data.joint_pos.torch[0, ids].detach().cpu().numpy()
        tcp, rotation = ik.tcp_pose(q[:6], station.BASES[side])
        result.append((tcp, rotation, q[6:]))
    return result


def jaws_clear(poses, grips):
    clouds = [pose.world(points) for pose, points in zip(poses, HULLS)]
    for tcp, rotation, fingers in grips:
        box = tcp + (SIGNS * [max(abs(fingers))+.013, .039, .033] + [0., 0., -.016]) @ rotation.T
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
    value, predicates = evaluate(*poses, TOP, WORK_X, False)
    current = np.array([p.position for p in poses]), np.array([p.rotation for p in poses])
    path = os.environ.get("SLEEVE_DIAGNOSTICS")
    grips = grip_state() if path or value == .75 else ()
    if value == .75 and predicates["placed"] and jaws_clear(poses, grips):
        while DWELL and not all(steady(old, current) for old in DWELL):
            DWELL.pop(0)
        DWELL.append(current)
        del DWELL[:-20]
    else:
        DWELL.clear()
    LAST_STEP, LAST_SCORE = STEP, 1. if len(DWELL) == 20 else value
    if path:
        row = dict(step=STEP, score=LAST_SCORE, dwell=len(DWELL), **predicates,
                   positions=current[0].tolist(), rotations=current[1].tolist(),
                   tcp=[p.tolist() for p, _, _ in grips], fingers=[f.tolist() for _, _, f in grips],
                   arm_contacts=CONTACTS.contacts if CONTACTS is not None else None,
                   grasp_substeps=GRASP_COUNTS.tolist())
        with open(path, "a") as log:
            log.write(json.dumps(row) + "\n")
    return LAST_SCORE


def success(objects):
    return score(objects) == 1.


def demo():
    def build(side, rows):
        skew = float(os.environ.get("SLEEVE_CONTROL_SKEW_DEG", "0")) if side == 1 else 0.
        result = []
        for index, (duration, x, y, z, finger, label) in enumerate(rows):
            yaw = -side * math.pi/2
            if index >= 4:
                yaw -= math.radians(1.7 if side == 1 else 2.0)
            if label in ("align front", "insert slowly", "seat flush") and skew:
                yaw += math.radians(skew)
            # Turning the yaw about the grasp tabs shifts the bodies in opposite x directions.
            if side == 1 and index >= 4:
                x += .003 if index in (6, 7) else .001
            correction = 1.45 if side == 1 and index in (5, 6, 7, 8, 9, 10, 11) else 0.
            lean = axis_angle_matrix(np.array([1., 0., 0.]), -side * math.radians(8 + correction))
            roll = axis_angle_matrix(np.array([0., 1., 0.]), math.radians(-.8 if correction else 0.))
            orientation = as_waypoint_rotation(roll @ lean @ grip_rotation(yaw))
            result.append(Waypoint(duration, (x, y + side*.003, TOP+z), finger, label, orientation))
        return result

    t, s = TRAY_HANDLE_Y, SLEEVE_HANDLE_Y
    a, b = TRAY_GRASP_Z, SLEEVE_GRASP_Z
    closed, open_ = FINGER_CLOSED, FINGER_OPEN
    higher_control = os.environ.get("SLEEVE_CONTROL_HIGH_ENTRY") == "1"
    alignment_dx, feed_dx, entry_lift = (0., 0., .010) if higher_control else (.001, .0005, .0085)
    left = (
        (3.0, HOME_X, TRAY_HOME_Y+t, .162, open_, "approach tray"),
        (1.6, HOME_X, TRAY_HOME_Y+t, a, open_, "lower over handle"),
        (1.0, HOME_X, TRAY_HOME_Y+t, a, closed, "grasp loaded tray"),
        (1.2, HOME_X, TRAY_HOME_Y+t, a+.024, closed, "lift tray"),
        (2.1, INSERT_X, .145+t, a+.024, closed, "align tray axis"),
        (1.5, INSERT_X+alignment_dx, .145+t, a+entry_lift, closed, "align front"),
        (5.0, INSERT_X+feed_dx, INSERT_Y+t+.008, a+entry_lift, closed, "insert slowly"),
        (1.2, INSERT_X, INSERT_Y+t-.003, a+.008, closed, "seat flush"),
        (1.3, INSERT_X, INSERT_Y+t-.003, a+.026, closed, "lift assembled package"),
        (2.2, WORK_X, .005+t-.003, a+.026, closed, "carry to work zone"),
        (1.5, WORK_X, .005+t-.003, a+.004, closed, "place assembled package"),
        (.9, WORK_X, .005+t-.003, a+.004, open_, "release tray"),
        (2.0, WORK_X, .005+t-.003, .155, open_, "retreat"),
    )
    right = (
        (3.0, HOME_X, SLEEVE_HOME_Y+s, .162, open_, "approach sleeve"),
        (1.6, HOME_X, SLEEVE_HOME_Y+s, b, open_, "lower over handle"),
        (1.0, HOME_X, SLEEVE_HOME_Y+s, b, closed, "grasp sleeve"),
        (1.2, HOME_X, SLEEVE_HOME_Y+s, b+.020, closed, "lift sleeve"),
        (2.1, INSERT_X, INSERT_Y+s, b+.020, closed, "align sleeve axis"),
        (1.5, INSERT_X, INSERT_Y+s, b, closed, "set down while counterholding"),
        (6.2, INSERT_X, INSERT_Y+s, b, closed, "counterhold during insertion"),
        (1.3, INSERT_X, INSERT_Y+s, b+.021, closed, "lift assembled package"),
        (2.2, WORK_X, s, b+.021, closed, "carry to work zone"),
        (1.5, WORK_X, s, b, closed, "place assembled package"),
        (.9, WORK_X, s, b, open_, "release sleeve"),
        (2.0, WORK_X, s, .155, open_, "retreat"),
    )
    return {"left": build(1, left), "right": build(-1, right)}
