"""Separate a connector after physically moving both of its interlocking locks."""

import json
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from secondary_lock_checks import ARM_NAMES, SIGNS, Dwell, Unlock, clear, evaluate, vertices, world
from secondary_lock_contacts import Contacts
from secondary_lock_motion import LEFT_YAW, RIGHT_YAW
from secondary_lock_physics import Part, Slider, env, on_reset, on_step
from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import Box, TABLE_TOP

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = .05
INSTRUCTION = "Release both connector locks and separate the plug from the socket without breaking either part."
TOP = TABLE_TOP[SCENE]
X = .310
START = (X, 0., TOP + .021)
PLUG_TARGET = np.array([X, -.110, TOP + .021])
SOCKET_TARGET = np.array([X, .100, TOP + .021])
SOCKET_GRIP = np.array([0., .048, .010])
PLUG_GRIP = np.array([-.002, -.049, .012])
SLIDER_GRIP = np.array([-.005, -.108, .032])
TEAL, NAVY, ORANGE = (.12, .49, .55), (.16, .31, .47), (.91, .42, .11)
YELLOW, RED, GREEN, WHITE = (.98, .68, .12), (.75, .17, .17), (.20, .50, .30), (.93, .93, .89)


def box(x0, x1, y0, y1, z0, z1, color):
    return (((x0 + x1)/2000, (y0 + y1)/2000, (z0 + z1)/2000),
            ((x1 - x0)/1000, (y1 - y0)/1000, (z1 - z0)/1000), color)


SOCKET = Part("socket", pos=START, size=(.046, .069, .042), mass=.082, color=TEAL, friction=.80,
              boxes=(box(-23, 23, 0, 24, -18, -8, TEAL),
                     box(-23, -11, 0, 24, -8, 18, TEAL),
                     box(17, 23, 0, 24, -8, 18, TEAL),
                     box(-11, 17, 0, 24, 9, 18, TEAL),
                     box(10.5, 17, 4, 9, -8, 8, NAVY),
                     box(-23, 23, 24, 31, -18, 18, TEAL),
                     box(-13, 13, 31, 69, -18, 18, TEAL)))
LATCH = Slider("latch", rest=(.014, -.032, .002), travel=.0067, rate=390., damping=3.2, mass=.008,
               axis="X", friction=.45,
               boxes=(box(12, 14.4, 11, 16, -5, 6, YELLOW),
                      box(7.6, 10.0, -31, 16, 3, 6, YELLOW),
                      box(9.8, 12.2, 11, 16, 3, 6, YELLOW),
                      box(8, 23, -59, -32, -4, 17, YELLOW),
                      box(8, 12, -33, -23, 3, 14, YELLOW),
                      box(-3, 10, -26, -14, 11, 16, YELLOW)))
SECONDARY = Slider("safety", rest=(-.005, -.060, .017), travel=.012, rate=0., damping=2.8,
                   mass=.008, axis="Y", friction=.50,
                   boxes=(box(-8.5, -3.6, -24, -15.5, 10.5, 17.5, RED),
                          box(-9, -4, -102, -24, 15, 18, RED),
                          box(-16, 6, -119, -96, 18, 34, RED),
                          box(-16, 6, -119, -116, 34, 35, WHITE)))
PLUG = Part("plug", pos=START, size=(.044, .135, .053), mass=.081, color=ORANGE, friction=.80,
            boxes=(box(-20, 20, -25, 0, -18, 10, ORANGE),
                   box(-17, 8.5, -69, -25, -18, 9, ORANGE),
                   box(-17, -10, -64, -36, 9, 16, ORANGE),
                   box(-9, 7.6, 0, 20, -6, 3, ORANGE),
                   box(-9, 7.6, 0, 20, 3, 7, ORANGE),
                   box(6.7, 7.6, 3, 10, -6, 7, ORANGE),
                   box(-17, 8, -32, -29, 10, 12, WHITE)),
            sliders=(LATCH, SECONDARY))
PADS = [
    Box("central_rest", pos=(X, -.018, TOP + .0015), size=(.078, .186, .003), color=GREEN),
    Box("socket_pad", pos=(X, .133, TOP + .0015), size=(.078, .100, .003), color=GREEN),
    Box("plug_pad", pos=(X, -.195, TOP + .0015), size=(.078, .160, .003), color=GREEN),
]
OBJECTS = [SOCKET, PLUG, *PADS]
PARTS = (PLUG, SOCKET)
SHAPES = tuple(vertices(part) for part in PARTS)
STEP, PREVIOUS_STEP, SCORE = 0, -1, 0.
CONTACTS = None
DWELL = Dwell()
LOCK = Unlock()


@on_reset
def reset(environment):
    global STEP, PREVIOUS_STEP, SCORE, CONTACTS, LOCK
    from isaaclab_physx.physics.physx_manager import PhysxManager

    STEP, PREVIOUS_STEP, SCORE = 0, -1, 0.
    LOCK = Unlock()
    DWELL.poses.clear()
    CONTACTS = Contacts(("plug", "socket", "safety", "latch"), os.environ.get("INTERLOCK_CONTACTS"))
    if not getattr(PhysxManager.step, "_interlock_contacts", False):
        native_step = PhysxManager.step.__func__

        def step(cls):
            native_step(cls)
            CONTACTS.poll(cls.get_physics_dt())

        step._interlock_contacts = True
        PhysxManager.step = classmethod(step)


@on_step
def tick(environment):
    global STEP
    STEP += 1


def jaws():
    base = env().unwrapped
    for arm in ARM_NAMES:
        robot = base.scene[f"{arm}_robot"]
        ids, _ = robot.find_joints([*ARM_JOINTS, *FINGER_JOINTS], preserve_order=True)
        q = robot.data.joint_pos.torch[0, ids].detach().cpu().numpy()
        center, rotation = ik.tcp_pose(q[:6], station.BASES[arm])
        fingers = center + (SIGNS * [max(abs(q[6:])) + .013, .039, .033] + [0., 0., -.016]) @ rotation.T
        yield fingers.min(0), fingers.max(0)


def stage_shape(slider, shift):
    return np.concatenate([np.array(center) + shift + SIGNS * np.array(size) / 2
                           for center, size, _ in slider.boxes])


def score(objects):
    global PREVIOUS_STEP, SCORE
    if PREVIOUS_STEP == STEP:
        return SCORE
    plug, socket = (objects[part.name] for part in PARTS)
    relative = (plug.pos - socket.pos) @ socket.rotation()
    withdrawal = float(-relative[1])
    travel, depression = -plug.sliders["safety"], -plug.sliders["latch"]
    alignment = (socket.rotation().T @ plug.rotation())[1, 1]
    aligned = alignment > .985 and np.linalg.norm(relative[[0, 2]]) < .006
    slider_open, tooth_clear = LOCK.update(travel, depression, withdrawal, aligned)
    physical_shapes = [
        np.concatenate((SHAPES[0], stage_shape(LATCH, [-depression, 0., 0.]),
                        stage_shape(SECONDARY, [0., -travel, 0.]))), SHAPES[1],
    ]
    shapes = [world(state, shape) for state, shape in zip((plug, socket), physical_shapes)]
    plug_front = np.max((shapes[0] - socket.pos) @ socket.rotation(), axis=0)[1]
    separated = plug_front < -.030
    on_pads = all(np.linalg.norm((state.pos - target)[:2]) < .021
                  and abs(shape[:, 2].min() - TOP - .003) < .002
                  for target, state, shape in zip((PLUG_TARGET, SOCKET_TARGET), (plug, socket), shapes))
    settled = all(np.linalg.norm(state.lin_vel) < .015 and np.linalg.norm(state.ang_vel) < .35
                  for state in (plug, socket))
    LOCK.invalid |= any(shape[:, 2].min() < TOP - .045 for shape in shapes)
    hands_clear = clear(list(jaws()), shapes)
    dwell = DWELL.update(separated and on_pads and settled and hands_clear, (plug, socket))
    SCORE = evaluate(slider_open, tooth_clear, withdrawal, LOCK.escaped, separated, on_pads, dwell, LOCK.invalid)
    PREVIOUS_STEP = STEP
    path = os.environ.get("INTERLOCK_DIAGNOSTICS")
    if path:
        row = dict(step=STEP, score=SCORE, withdrawal_m=withdrawal, safety_m=travel, latch_m=depression,
                   plug_front_m=float(plug_front), escaped=bool(LOCK.escaped), staged=bool(on_pads),
                   dwell=dwell, hands_clear=hands_clear, invalid=bool(LOCK.invalid),
                   position={part.name: state.pos.tolist() for part, state in zip(PARTS, (plug, socket))},
                   rotation={part.name: state.rotation().tolist() for part, state in zip(PARTS, (plug, socket))},
                   fingers_n=CONTACTS.loads.tolist(), arm_contacts=CONTACTS.count)
        with open(path, "a") as stream:
            stream.write(json.dumps(row) + "\n")
    return SCORE


def success(objects):
    return score(objects) == 1.


def xyz(values):
    return tuple(float(value) for value in values)


def demo():
    mode = os.environ.get("INTERLOCK_DEMO", "demo")
    if mode == "idle":
        return {}
    left, right = [], []
    opened, shut = FINGER_OPEN, FINGER_CLOSED
    start = np.array(START)
    ls = start + SOCKET_GRIP
    rs = start + SLIDER_GRIP + [.0025, 0., 0.]
    rp = start + PLUG_GRIP
    high = np.array([0., 0., .071])
    lift = np.array([0., 0., .024])

    def both(seconds, lpos, rpos, lfingers, rfingers, label):
        left.append(Waypoint(seconds, xyz(lpos), lfingers, label, LEFT_YAW))
        right.append(Waypoint(seconds, xyz(rpos), rfingers, label, RIGHT_YAW))

    if mode == "locked":
        rear_housing = start + [-.00425, -.073, .006]
        both(2., ls + high, rear_housing + high, opened, opened, "approach the coupled housings")
        both(1.4, ls, rear_housing, opened, opened, "descend over the rear housing and socket")
        both(.9, ls, rear_housing, shut, shut, "hold the plug housing and counterhold the socket")
        both(1.9, ls, rear_housing + [0., -.032, 0.], shut, shut, "pull directly against the locked shoulder")
        both(1.1, ls, rear_housing, shut, shut, "unload the blocked pull")
        both(.9, ls, rear_housing, shut, opened, "release the plug housing")
        both(1.4, ls, rear_housing + high, shut, opened, "retreat the right arm while counterholding")
        both(.8, ls, rear_housing + high, opened, opened, "release the socket")
        both(1.4, ls + high, rear_housing + high, opened, opened, "retreat from the coupled connector")
        return {"left": left, "right": right}

    both(2., ls + high, rs + high, opened, opened, "approach socket and safety tab")
    both(1.4, ls, rs, opened, opened, "descend over both grips")
    both(.9, ls, rs, shut, shut, "counterhold socket and grip safety tab")
    stroke = 0. if mode == "locked_latch" else .006 if mode == "half" else .0125
    retracted = rs + [.004 * stroke / .0125, -stroke, 0.]
    both(1.4, ls, retracted, shut, shut, "withdraw the secondary safety")
    both(.8, ls, retracted, shut, opened, "release the safety tab")
    both(1.2, ls, retracted + high, shut, opened, "lift above the safety tab")
    both(1.2, ls, rp + high, shut, opened, "approach the main latch collar")
    both(1.4, ls, rp, shut, opened, "descend on the main latch")
    both(1., ls, rp, shut, shut, "compress the latch against the plug")
    if mode in ("locked_latch", "half"):
        demand = rp + [0., -.032, 0.]
        relieved = -.020
        behind_collar = rp + [0., -.031, 0.]
        both(1.9, ls, demand, shut, shut, "apply the same withdrawal demand with the lock engaged")
        both(1.1, ls, rp, shut, shut, "unload the blocked pull")
        both(.9, ls, rp, shut, relieved, "relieve finger pressure without lifting the plug")
        both(1.6, ls, behind_collar, shut, relieved, "slide the relieved jaws past the rear collar edge")
        both(.8, ls, behind_collar, shut, opened, "open the jaws clear of the collar")
        both(1.4, ls, behind_collar + high, shut, opened, "retreat the right arm while counterholding")
        both(.8, ls, behind_collar + high, opened, opened, "open left jaws above the socket rest")
        both(1.4, ls + high, behind_collar + high, opened, opened, "retreat from the coupled connector")
        return {"left": left, "right": right}
    both(1.9, ls, rp + [0., -.032, 0.], shut, shut, "withdraw the plug past its shoulder")
    both(1.7, ls, rp + [0., -.066, 0.], shut, shut, "separate the mating faces")
    both(1.2, ls + lift, rp + [0., -.066, .024], shut, shut, "raise both separate halves")
    both(1.6, SOCKET_TARGET + SOCKET_GRIP + lift, PLUG_TARGET + PLUG_GRIP + lift,
         shut, shut, "carry the halves to separate rests")
    both(1.4, SOCKET_TARGET + SOCKET_GRIP, PLUG_TARGET + PLUG_GRIP, shut, shut, "place both halves")
    both(.9, SOCKET_TARGET + SOCKET_GRIP, PLUG_TARGET + PLUG_GRIP, opened, opened, "release both grips")
    both(1.6, SOCKET_TARGET + SOCKET_GRIP + high, PLUG_TARGET + PLUG_GRIP + high,
         opened, opened, "retreat clear of both components")
    return {"left": left, "right": right}
