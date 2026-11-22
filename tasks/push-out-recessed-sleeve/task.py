"""Extract a free recessed sleeve through an open bore with a blunt push rod."""

import json
import math
import os
import sys
from dataclasses import asdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sleeve_geometry import annulus
from sleeve_scoring import State, score as evaluate
from sleevekit import Part, env, on_reset, on_step
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import Box, TABLE_TOP

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = .05
INSTRUCTION = "Push the recessed sleeve out of the plate into the catch tray using the blunt rod, then park the rod."
TOP = TABLE_TOP[SCENE]
BORE = (.315, -.143, TOP + .045)
ROD = (.180, -.143, TOP + .011)
CATCH = (.400, -.143)
CATCH_ENTRY_X = .330
PLATE_DIMENSIONS = (.020, .120, .090)
PLATE_BLUE = (.22, .46, .76)
SLEEVE_YELLOW = (.94, .68, .18)
ROD_RED = (.80, .23, .17)
GREY = (.30, .34, .41)
GREEN = (.21, .55, .32)
GUIDE_Y = .082

# The left grasp is raised and displaced toward the left arm, clear of the bore.
GRIP_XY = (BORE[0] - .0045, BORE[1] + .168)
PLATE = Part("plate", pos=BORE, size=PLATE_DIMENSIONS, mass=.19, color=PLATE_BLUE,
             wedges=annulus(.020, .012, outer_half=(.060, .045)),
             boxes=(((0., .111, .031), (.020, .110, .022), PLATE_BLUE),
                    ((0., .168, .044), (.025, .024, .056), PLATE_BLUE),
                    ((0., .067, -.0395), (.020, .018, .005), PLATE_BLUE),
                    ((0., -.067, -.0395), (.020, .018, .005), PLATE_BLUE)))
SLEEVE = Part("sleeve", pos=BORE, size=(.016, .0238, .0238), mass=.011,
              color=SLEEVE_YELLOW, wedges=annulus(.016, .005, outer_radius=.0119))
TOOL = Part("rod", pos=ROD, size=(.113, .025, .065), mass=.045, color=ROD_RED, friction=.9,
            cylinders=(((0., 0., 0.), .007, .100, "X", ROD_RED),),
            boxes=(((-.048, 0., .022), (.025, .025, .055), ROD_RED),))


def fixed(name, pos, size, color=GREY):
    return Box(name, pos=pos, size=size, color=color)


OBJECTS = [PLATE, SLEEVE, TOOL,
           fixed("guide_left", (.309, BORE[1] + GUIDE_Y, TOP + .006), (.060, .010, .012)),
           fixed("guide_right", (.309, BORE[1] - GUIDE_Y, TOP + .006), (.060, .010, .012)),
           fixed("guide_left_lip", (.309, BORE[1] + .075, TOP + .011), (.060, .016, .004)),
           fixed("guide_right_lip", (.309, BORE[1] - .075, TOP + .011), (.060, .016, .004)),
           fixed("rod_rest", (ROD[0], ROD[1], TOP + .002), (.133, .070, .004), GREEN),
           fixed("rod_guide_left", (ROD[0] + .022, ROD[1] + .013, TOP + .007), (.035, .008, .014)),
           fixed("rod_guide_right", (ROD[0] + .022, ROD[1] - .013, TOP + .007), (.035, .008, .014)),
           fixed("catch_floor", ((CATCH_ENTRY_X + CATCH[0] + .052) / 2, CATCH[1], TOP + .002),
                 (CATCH[0] + .052 - CATCH_ENTRY_X, .083, .004), GREEN),
           fixed("catch_rear", (CATCH[0] + .052, CATCH[1], TOP + .016), (.004, .087, .032), GREEN),
           fixed("catch_left", (CATCH[0], CATCH[1] + .0415, TOP + .016), (.104, .004, .032), GREEN),
           fixed("catch_right", (CATCH[0], CATCH[1] - .0415, TOP + .016), (.104, .004, .032), GREEN)]

SIGNS = np.array([(x, y, z) for x in [-1, 1] for y in [-1, 1] for z in [-1, 1]])
SLEEVE_POINTS = np.concatenate(SLEEVE.wedges)
TOOL_POINTS = np.concatenate((SIGNS * [.05, .007, .007],
                              np.array([-.048, 0., .022]) + SIGNS * [.0125, .0125, .0275]))
PLATE_POINTS = np.concatenate((np.concatenate(PLATE.wedges),
                               *(np.array(c) + SIGNS * np.array(s) / 2 for c, s, _ in PLATE.boxes)))
STEP = 0
SCORE_STEP = -1
SCORE = 0.
STABLE_POSES = []
LOST = False
CONTACTS = None


@on_reset
def reset(environment):
    global STEP, SCORE_STEP, SCORE, LOST, CONTACTS
    STEP, SCORE_STEP, SCORE, LOST = 0, -1, 0., False
    STABLE_POSES.clear()
    path = os.environ.get("SLEEVE_ARM_CONTACTS")
    if path:
        from sleeve_contacts import ArmContacts
        from isaaclab_physx.physics.physx_manager import PhysxManager

        CONTACTS = ArmContacts(path)
        if not getattr(PhysxManager.step, "_sleeve_contacts", False):
            native_step = PhysxManager.step.__func__

            def step(cls):
                native_step(cls)
                CONTACTS.poll(cls.get_physics_dt())

            step._sleeve_contacts = True
            PhysxManager.step = classmethod(step)


@on_step
def tick(environment):
    global STEP
    STEP += 1


def transformed(state, points):
    return points @ state.rotation().T + state.pos


def stable(previous, current):
    previous_positions, previous_rotations = previous
    positions, rotations = current
    distance = np.linalg.norm(positions - previous_positions, axis=1)
    cosine = (np.sum(previous_rotations * rotations, axis=(1, 2)) - 1) / 2
    return bool(np.all(distance <= .002) and np.all(cosine >= math.cos(math.radians(2))))


def hands_clear(*parts):
    from yam_arena import ik, station

    base = env().unwrapped
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        ids, _ = robot.find_joints([*ARM_JOINTS, *FINGER_JOINTS], preserve_order=True)
        q = robot.data.joint_pos.torch[0, ids].detach().cpu().numpy()
        center, rot = ik.tcp_pose(q[:6], station.BASES[arm])
        fingers = center + (SIGNS * [max(abs(q[6:])) + .013, .039, .033] + [0., 0., -.016]) @ rot.T
        low, high = fingers.min(0), fingers.max(0)
        for points in parts:
            gap = np.maximum(points.min(0) - high, low - points.max(0)).max()
            if gap < .010:
                return False
    return True


def measure(objects):
    plate, sleeve, rod = (objects[name] for name in ("plate", "sleeve", "rod"))
    sleeve_world = transformed(sleeve, SLEEVE_POINTS)
    rod_world = transformed(rod, TOOL_POINTS)
    plate_world = transformed(plate, PLATE_POINTS)
    sleeve_local = (sleeve_world - plate.pos) @ plate.rotation()
    low, high = sleeve_world.min(0), sleeve_world.max(0)
    travel = float(((sleeve.pos - plate.pos) @ plate.rotation())[0])
    min_x, max_x = sleeve_local[:, 0].min(), sleeve_local[:, 0].max()
    outside = min_x >= .0105 or (float(high[2]) < plate.pos[2] - .014 and travel > .010)
    floor = TOP + .004
    in_tray = bool(np.all(low[:2] >= [CATCH_ENTRY_X - .0005, CATCH[1] - .040])
                   and np.all(high[:2] <= np.asarray(CATCH) + [.050, .0395] + .0005)
                   and abs(low[2] - floor) < .002)
    # The parked tool must be on its green rest with the shaft aligned along x.
    rod_parked = (np.linalg.norm(rod.pos[:2] - ROD[:2]) < .014
                  and rod.rotation()[0, 0] > math.cos(math.radians(12))
                  and abs(rod_world[:, 2].min() - floor) < .002)
    rod_in_plate = (rod_world - plate.pos) @ plate.rotation()
    empty_bore = bool(outside and rod_in_plate[:, 0].max() < -.011)
    plate_stable = (np.linalg.norm(plate.pos - BORE) < .016 and plate.rotation()[2, 2] > .985
                    and plate.rotation()[0, 0] > .985 and abs(plate_world[:, 2].min() - TOP) < .002)
    settled = all(np.linalg.norm(obj.lin_vel) < .013 and np.linalg.norm(obj.ang_vel) < .35
                  for obj in (plate, sleeve, rod))
    released = bool(in_tray and rod_parked and plate_stable and empty_bore and settled
                    and hands_clear(plate_world, sleeve_world, rod_world))
    lost = bool(low[2] < TOP - .050)
    state = State(travel=travel, rear_projection=float(max_x - .010), outside=bool(outside), in_tray=in_tray,
                  empty_bore=empty_bore, rod_parked=bool(rod_parked), plate_stable=bool(plate_stable), lost=lost)
    positions = {name: objects[name].pos.tolist() for name in ("plate", "sleeve", "rod")}
    rotations = {name: objects[name].rotation().tolist() for name in ("plate", "sleeve", "rod")}
    return state, released, positions, rotations


def score(objects):
    global SCORE_STEP, SCORE, LOST
    if STEP == SCORE_STEP:
        return SCORE
    state, released, positions, rotations = measure(objects)
    LOST = LOST or state.lost
    if released:
        sample = (np.array(list(positions.values())), np.array(list(rotations.values())))
        while STABLE_POSES and not all(stable(old, sample) for old in STABLE_POSES):
            STABLE_POSES.pop(0)
        STABLE_POSES.append(sample)
        STABLE_POSES[:] = STABLE_POSES[-20:]
    else:
        STABLE_POSES.clear()
    flags = dict(asdict(state), released=(len(STABLE_POSES) == 20), lost=LOST)
    SCORE_STEP, SCORE = STEP, evaluate(State(**flags))
    path = os.environ.get("SLEEVE_DIAGNOSTICS")
    if path:
        result = dict(step=STEP, score=SCORE, dwell=len(STABLE_POSES), poses=positions, rotation=rotations, **flags)
        if CONTACTS is not None:
            result["arm_contacts"] = CONTACTS.contacts
            result["left_grasp"] = CONTACTS.read_grasp(objects["plate"])
        with open(path, "a") as log:
            log.write(json.dumps(result) + "\n")
    return SCORE


def success(objects):
    return score(objects) == 1.


def demo():
    from sleeve_pose import ready_pose

    mode = os.environ.get("SLEEVE_DEMO")
    if mode == "idle":
        return {}
    left, right = [], []
    lyaw, ryaw = -math.pi / 2, math.pi / 2
    opened, closed = FINGER_OPEN, FINGER_CLOSED
    hover, lgrip = TOP + .178, (*GRIP_XY, TOP + .094)
    rod_x = ROD[0] - .048
    rod_grip = (rod_x, ROD[1], TOP + .045)
    parked, parked_yaw = ready_pose("right")
    for seconds, p, grip, label in [(2., (*GRIP_XY, hover), opened, "left approach"),
                                     (1.6, lgrip, opened, "counterhold descent"),
                                     (.8, lgrip, closed, "grip plate")]:
        left.append(Waypoint(seconds, p, grip, label, lyaw))
        right.append(Waypoint(seconds, parked, opened, "right at ready", parked_yaw))

    def both(seconds, rpos, rgrip, label, lpos=lgrip, lfingers=closed):
        left.append(Waypoint(seconds, lpos, lfingers, label, lyaw))
        right.append(Waypoint(seconds, rpos, rgrip, label, ryaw))

    both(2., (rod_x, ROD[1], TOP + .105), opened, "approach rod")
    both(1.5, rod_grip, opened, "descend to rod")
    both(.9, rod_grip, closed, "grip rod")
    work_z = rod_grip[2] + BORE[2] - ROD[2] + .0015
    push_y = BORE[1] - .0045
    both(1.4, (rod_x, BORE[1], work_z), closed, "raise rod to bore axis")
    both(1.8, (BORE[0] - .110, push_y, work_z), closed, "align blunt tip")
    both(2.4, (BORE[0] - .080, push_y, work_z), closed, "push sleeve entirely through")
    both(1.6, (BORE[0] - .134, push_y, work_z), closed, "withdraw rod")
    both(1., (rod_x, ROD[1], work_z), closed, "return rod over rest")
    both(1.3, rod_grip, closed, "lower rod to rest")
    both(.8, rod_grip, opened, "release both grips", lfingers=opened)
    both(1.8, (rod_x, ROD[1], TOP + .110), opened, "lift both jaws", lpos=(*GRIP_XY, hover), lfingers=opened)
    if mode == "no_counterhold":
        return {"right": right}
    return {"left": left, "right": right}
