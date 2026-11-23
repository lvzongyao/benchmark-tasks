"""Remove the shipping spacer while manually supporting a captive sprung carrier."""

import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_contacts import ArmContacts
from native_joints import JOINTS, Joint
from rigid_parts import Part, env, on_reset, on_step
from scoring import CLEAR_TRAVEL, Trial, UPPER
from wrist_pose import as_waypoint_rotation
from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint, grip_rotation
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = .05
INSTRUCTION = "Remove the shipping block while supporting the carrier, then lower the carrier gently onto its stop."
TOP = TABLE_TOP[SCENE]
BASE = np.array([.277, 0., TOP])
STOP = .021
BLOCK_RESET_Y = -.008
BLOCK_RESET = BASE + [0., BLOCK_RESET_Y, STOP]
TRAY = np.array([.397, -.216, TOP])
BLOCK_HANDLE = np.array([.023, -.095, .026])
CARRIER_HANDLE = np.array([-.022, .035, .044])
NAMES = ("fixture_base", "spring_carrier", "shipping_spacer", "spacer_tray")
BLUE, CARRIER, AMBER, DARK = (.22, .34, .44), (.51, .66, .70), (.94, .62, .23), (.28, .40, .43)
GREEN, PINK = (.36, .70, .51), (.86, .41, .34)
SIGNS = np.array([(x, y, z) for x in (-1., 1.) for y in (-1., 1.) for z in (-1., 1.)])


def xyz(values):
    return tuple(float(value) for value in values)


def cube(x0, x1, y0, y1, z0, z1, color):
    low, high = np.array((x0, y0, z0)), np.array((x1, y1, z1))
    return xyz((low+high)/2), xyz(high-low), color


def fixture():
    shapes = [
        cube(-.070, .070, -.050, .050, 0., .012, BLUE),
        cube(-.031, .031, -.049, .049, .012, STOP, DARK),
        cube(-.048, -.037, -.032, .032, .012, STOP, GREEN),
        cube(.037, .048, -.032, .032, .012, STOP, GREEN),
    ]
    for sign in (-1, 1):
        x = sign*.061
        shapes.extend((
            cube(x-.006, x+.006, -.036, .037, .012, .073, BLUE),
            cube(x-.007, x+.007, -.038, -.034, STOP-.001, STOP+.002, GREEN),
            cube(x-.007, x+.007, -.038, -.034, STOP+UPPER-.001, STOP+UPPER+.002, PINK),
        ))
    return tuple(shapes)


def carrier():
    return (
        cube(-.050, .050, -.035, .035, 0., .015, CARRIER),
        cube(-.040, -.004, .010, .057, .015, .058, CARRIER),
        cube(-.047, .003, .008, .063, .050, .062, CARRIER),
        cube(-.052, -.048, -.032, -.027, 0., .003, AMBER),
        cube(.048, .052, -.032, -.027, 0., .003, AMBER),
    )


def spacer():
    return (
        cube(-.029, .029, -.029, .029, 0., UPPER, AMBER),
        cube(-.007, .041, -.116, -.028, .008, .017, AMBER),
        cube(.006, .040, -.118, -.072, .016, .043, AMBER),
    )


def tray():
    return (
        cube(-.052, .052, -.107, .068, 0., .004, DARK),
        cube(-.052, -.048, -.107, .068, .004, .013, GREEN),
        cube(.048, .052, -.107, .068, .004, .013, GREEN),
        cube(-.048, .048, -.107, -.103, .004, .013, GREEN),
        cube(-.048, .048, .064, .068, .004, .013, GREEN),
    )


OBJECTS = [
    Part(NAMES[0], pos=xyz(BASE), size=(.140, .100, .073), mass=.40, boxes=fixture(), friction=.58),
    Part(NAMES[1], pos=xyz(BASE+[0., 0., STOP+UPPER]), size=(.104, .080, .062), mass=.115,
         boxes=carrier(), friction=.54),
    Part(NAMES[2], pos=xyz(BLOCK_RESET), size=(.058, .133, .043), mass=.073,
         boxes=spacer(), friction=.46),
    Part(NAMES[3], pos=xyz(TRAY), size=(.104, .175, .013), mass=.21, boxes=tray(), friction=.66),
]
JOINTS.extend((
    Joint("base_mount", "fixed", None, NAMES[0], xyz(BASE), (0.,0.,0.), collisions=False),
    Joint("carrier_guide", "prismatic", NAMES[0], NAMES[1], (0.,0.,STOP), (0.,0.,0.), axis="Z",
          limits=(-.002, .040), collisions=True, spring=(22., .40, -.016)),
))
HULLS = tuple(np.concatenate([np.array(c)+SIGNS*np.array(s)/2 for c, s, _ in p.boxes]) for p in OBJECTS)
TRIAL = Trial()
STEP, LAST_STEP, LAST_SCORE = 0, -1, 0.
DWELL = []
ARM_CONTACTS = None
CARRIER_VIEW = BLOCK_VIEW = BASE_CONTACTS = TRAY_CONTACTS = GRASP_VIEW = None
GRASP_COUNTS = np.zeros((4, 2), dtype=int)


@on_reset
def reset(environment):
    import carb
    from isaaclab.sim.utils.stage import get_current_stage
    from isaaclab_physx.physics.physx_manager import PhysxManager

    global TRIAL, STEP, LAST_STEP, LAST_SCORE, ARM_CONTACTS
    global CARRIER_VIEW, BLOCK_VIEW, BASE_CONTACTS, TRAY_CONTACTS, GRASP_VIEW
    TRIAL = Trial()
    STEP, LAST_STEP, LAST_SCORE = 0, -1, 0.
    DWELL.clear()
    GRASP_COUNTS.fill(0)
    carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    paths = [prim.GetPath().pathString for prim in get_current_stage().Traverse()]
    fixture_path, carrier_path, block_path, tray_path = (
        next(p for p in paths if p.lower().endswith(f"/{name}/body")) for name in NAMES
    )
    physics = PhysxManager.get_physics_sim_view()
    CARRIER_VIEW = physics.create_rigid_body_view(carrier_path)
    BLOCK_VIEW = physics.create_rigid_body_view(block_path)
    BASE_CONTACTS = physics.create_rigid_contact_view(
        [carrier_path], filter_patterns=[[fixture_path, block_path]], max_contact_data_count=96)
    TRAY_CONTACTS = physics.create_rigid_contact_view([block_path], filter_patterns=[[tray_path]], max_contact_data_count=64)
    path = os.environ.get("CARRIER_ARM_CONTACTS")
    ARM_CONTACTS = ArmContacts(path) if path else None
    if ARM_CONTACTS is not None:
        fingers = [name for name in ARM_CONTACTS.left+ARM_CONTACTS.right
                   if name.rsplit("/", 1)[-1] in ("tip_left", "tip_right")]
        GRASP_VIEW = physics.create_rigid_contact_view(fingers, filter_patterns=[[carrier_path, block_path]]*4,
                                                        max_contact_data_count=128)
    if not getattr(PhysxManager.step, "_d2_speed", False):
        native = PhysxManager.step.__func__

        def step(cls):
            native(cls)
            dt = cls.get_physics_dt()
            carrier_z = float(np.asarray(CARRIER_VIEW.get_transforms().numpy())[0, 2])
            carrier_vz = float(np.asarray(CARRIER_VIEW.get_velocities().numpy())[0, 2])
            block_y = float(np.asarray(BLOCK_VIEW.get_transforms().numpy())[0, 1])
            TRIAL.sample(-carrier_vz, carrier_z-BASE[2]-STOP, BLOCK_RESET_Y-block_y, dt)
            if ARM_CONTACTS is not None:
                ARM_CONTACTS.poll(dt)
                forces = np.asarray(GRASP_VIEW.get_contact_force_matrix(dt=dt).numpy())
                GRASP_COUNTS[:] += np.linalg.norm(forces, axis=-1) > .1

        step._d2_speed = True
        PhysxManager.step = classmethod(step)


@on_step
def tick(environment):
    global STEP
    STEP += 1


def grippers():
    result = []
    for side in ("left", "right"):
        arm = env().unwrapped.scene[f"{side}_robot"]
        ids, _ = arm.find_joints([*ARM_JOINTS, *FINGER_JOINTS], preserve_order=True)
        q = arm.data.joint_pos.torch[0, ids].detach().cpu().numpy()
        tcp, orientation = ik.tcp_pose(q[:6], station.BASES[side])
        result.append((tcp, orientation, q[6:]))
    return result


def clear(hulls, grips):
    for tcp, ori, fingers in grips:
        box = tcp+(SIGNS*[max(abs(fingers))+.013, .039, .033]+[0., 0., -.016])@ori.T
        for hull in hulls:
            separation = np.maximum(hull.min(0)-box.max(0), box.min(0)-hull.max(0))
            if separation.max() < .010:
                return False
    return True


def score(objects):
    global LAST_STEP, LAST_SCORE
    from isaaclab_physx.physics.physx_manager import PhysxManager

    if STEP == LAST_STEP:
        return LAST_SCORE
    parts = [objects[name] for name in NAMES]
    positions = np.array([part.pos for part in parts])
    rotations = np.array([part.rotation() for part in parts])
    clouds = [p+h@r.T for p, r, h in zip(positions, rotations, HULLS)]
    base, carrier, block, tray = positions
    height = carrier[2]-base[2]-STOP
    travel = BLOCK_RESET[1]-block[1]
    block_in_tray = (clouds[2]-tray) @ rotations[3]
    dt = PhysxManager.get_physics_dt()
    base_load, spacer_load = np.asarray(BASE_CONTACTS.get_contact_force_matrix(dt=dt).numpy())[0, :, 2]
    tray_load = float(np.asarray(TRAY_CONTACTS.get_contact_force_matrix(dt=dt).numpy())[0, 0, 2])
    staged = bool(np.min(block_in_tray[:, 0]) >= -.049 and np.max(block_in_tray[:, 0]) <= .049
                  and np.min(block_in_tray[:, 1]) >= -.104 and np.max(block_in_tray[:, 1]) <= .065
                  and abs(np.min(block_in_tray[:, 2])-.004) <= .002 and tray_load > .4)
    block_cloud = clouds[2]
    clear_spacer = (np.max(block_cloud[:, 1]) <= base[1]-.039
                    or np.min(block_cloud[:, 1]) >= base[1]+.039
                    or np.max(block_cloud[:, 0]) <= base[0]-.053
                    or np.min(block_cloud[:, 0]) >= base[0]+.053)
    stopped = bool(abs(height) <= .0015 and base_load > 1.0 and spacer_load < .10)
    grips = grippers()
    free = clear(clouds[1:3], grips)
    valid = cloud_minimum(clouds[2]) >= TOP-.010 and abs(tray[2]-TOP) <= .006
    if valid and clear_spacer and staged and stopped and free and not TRIAL.dropped and not TRIAL.premature_descent:
        while DWELL and not all(np.all(np.linalg.norm(positions-p, axis=1) <= .002)
                                and np.all((np.sum(rotations*r, axis=(1, 2))-1)/2 >= math.cos(math.radians(2)))
                                for p, r in DWELL):
            DWELL.pop(0)
        DWELL.append((positions, rotations))
        del DWELL[:-20]
    else:
        DWELL.clear()
    value = TRIAL.score(height, travel, clear_spacer, staged, stopped, len(DWELL) == 20)
    LAST_STEP, LAST_SCORE = STEP, value if valid else 0.
    path = os.environ.get("CARRIER_DIAGNOSTICS")
    if path:
        info = dict(step=STEP, score=LAST_SCORE, dwell=len(DWELL), height_m=float(height),
                    travel_m=float(travel), peak_downward_speed_mps=TRIAL.peak_speed,
                    dropped=TRIAL.dropped, premature_descent=bool(TRIAL.premature_descent),
                    clear_spacer=bool(clear_spacer), supported_force_n=float(base_load),
                    spacer_force_n=float(spacer_load), tray_force_n=tray_load, staged=staged,
                    released=free, valid=bool(valid), positions=positions.tolist(), rotations=rotations.tolist(),
                    block_tray_bounds_m=[block_in_tray.min(0).tolist(), block_in_tray.max(0).tolist()],
                    tcp=[p.tolist() for p, _, _ in grips], fingers=[f.tolist() for _, _, f in grips],
                    arm_contacts=ARM_CONTACTS.contacts if ARM_CONTACTS is not None else None,
                    grasp_substeps=GRASP_COUNTS.tolist())
        with open(path, "a") as log:
            log.write(json.dumps(info)+"\n")
    return LAST_SCORE


def cloud_minimum(points):
    return np.min(points[:, 2])


def success(objects):
    return score(objects) == 1.


def demo():
    open_, closed = FINGER_OPEN, FINGER_CLOSED
    cp = BASE+[0.,0.,STOP+UPPER]+CARRIER_HANDLE
    sp = BLOCK_RESET+BLOCK_HANDLE
    dest = TRAY+[-.007, .029, .005]+BLOCK_HANDLE

    def above(position):
        return (position[0], position[1], TOP+.150)

    def make(side, rows):
        lean = axis_angle_matrix(np.array([1.,0.,0.]), math.radians(-6. if side == 1 else 0.))
        yaw = -math.pi/2 if side == 1 else 0.
        orientation = as_waypoint_rotation(lean @ grip_rotation(yaw))
        return [Waypoint(duration, xyz(np.asarray(pos)+[0., side*.002, 0.]), finger, label, orientation)
                for duration, pos, finger, label in rows]

    holder = cp+[0.,0.,.007]
    extracted = sp+[0.,-.100,0.]
    stop_pose = cp+[0.,0.,-UPPER-.001]
    left = (
        (3., above(cp), open_, "approach spring-biased carrier"),
        (1.8, cp, open_, "lower above carrier grip"),
        (.9, cp, closed, "take the carrier load"),
        (1.2, holder, closed, "lift the carrier to unload the spacer"),
        (12.1, holder, closed, "maintain the carrier height during spacer withdrawal and staging"),
        (3.8, stop_pose, closed, "lower carrier under control onto mechanical stop"),
        (1.0, stop_pose, open_, "release supported carrier"),
        (2.0, above(cp), open_, "retreat after descent"),
    )
    block_open = -.033
    right = (
        (2.4, above(sp+[0.,-.105,0.]), block_open, "approach the block from outside"),
        (.6, above(sp), block_open, "align over the shipping block"),
        (1.8, sp, block_open, "lower over block handle"),
        (.9, sp, block_open, "wait for carrier grip"),
        (1.2, sp, block_open, "wait until spacer unloads"),
        (.9, sp, closed, "grasp shipping block"),
        (.7, sp, closed, "steady the spacer before withdrawal"),
        (3.4, extracted, closed, "withdraw spacer horizontally"),
        (1.5, above(extracted), closed, "lift clear of the fixture"),
        (2.1, above(dest), closed, "carry spacer to recovery tray"),
        (1.6, dest, closed, "stage spacer in tray"),
        (.9, dest, open_, "release spacer"),
        (1.8, above(dest), open_, "retreat from staged spacer"),
    )
    plans = {"right": make(-1, right)}
    if os.environ.get("CARRIER_CONTROL") != "unsupported":
        plans["left"] = make(1, left)
    return plans
