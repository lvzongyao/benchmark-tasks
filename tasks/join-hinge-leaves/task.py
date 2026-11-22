"""Join a free flap to a base using a physical pin through three open eyes."""

import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hinge_geometry import eye, pin_nose
from hingekit import Part, env, on_reset, on_step
from hinge_scoring import HingeWitness, assess, flap_angle, score as evaluate
from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint, grip_rotation
from yam_arena.task_file import Box, TABLE_TOP

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = .05
INSTRUCTION = "Join the flap to the base with the pivot pin and show that the flap can turn while remaining attached."
TOP = TABLE_TOP[SCENE]
AXIS = np.array([.270, -.058, TOP + .064])
LEAF_START = np.array([.235, .145, TOP + .013])
PIN_START = np.array([.270, -.200, TOP + .008])
BLUE, ORANGE, RED, GREY = (.17,.43,.73), (.94,.52,.17), (.77,.18,.15), (.28,.32,.37)
LEAF_GRIP = np.array([.064, .090, .024])
PIN_GRIP = np.array([0., -.047, .028])


def xyz(point):
    return tuple(float(value) for value in point)


BASE = Part("hinge_base", pos=xyz(AXIS), size=(.130,.100,.077), mass=.46, color=BLUE, friction=.6,
            wedges=eye(.010,.0058,.013,-.0225) + eye(.010,.0063,.013,.0225),
            boxes=(((-.015,0.,-.060),(.130,.099,.008),BLUE),
                   ((-.006,-.0225,-.0335),(.014,.010,.052),BLUE),
                   ((-.006,.0225,-.0335),(.014,.010,.052),BLUE),
                   ((.045,0.,-.0315),(.012,.034,.049),GREY)))
FLAP = Part("flap", pos=xyz(LEAF_START), size=(.095,.133,.063), mass=.10, color=ORANGE,
            wedges=eye(.026,.0058,.013),
            boxes=(((.059,0.,-.002),(.080,.060,.010),ORANGE),
                   ((.014,0.,-.002),(.014,.024,.010),ORANGE),
                   ((.064,.054,.004),(.020,.068,.012),ORANGE),
                   (tuple(LEAF_GRIP),(.024,.028,.040),ORANGE)))
PIN = Part("pivot_pin", pos=xyz(PIN_START), size=(.025,.108,.048), mass=.032, color=RED, friction=.8,
           wedges=pin_nose(),
           cylinders=(((0.,-.002,0.),.005,.061,"Y",RED), ((0.,.0305,0.),.003,.004,"Y",RED)),
           boxes=(((0.,-.041,.0075),(.025,.018,.025),RED),
                  (tuple(PIN_GRIP),(.025,.024,.028),RED)))
OBJECTS = [BASE, FLAP, PIN,
           Box("leaf_stand", pos=xyz((LEAF_START[0]+.048,LEAF_START[1],TOP+.003)),
               size=(.050,.062,.006),color=GREY),
           Box("leaf_wing_stand", pos=xyz((LEAF_START[0]+.066,LEAF_START[1]+.066,TOP+.0055)),
               size=(.025,.047,.011),color=GREY),
           Box("pin_rest", pos=xyz((*PIN_START[:2],TOP+.0015)), size=(.058,.122,.003),color=GREY),
           Box("pin_guide_a", pos=xyz((PIN_START[0]-.009,PIN_START[1]+.016,TOP+.006)),
               size=(.006,.038,.012),color=GREY),
           Box("pin_guide_b", pos=xyz((PIN_START[0]+.009,PIN_START[1]+.016,TOP+.006)),
               size=(.006,.038,.012),color=GREY)]
SIGNS = np.array([(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)])


def vertices(spec):
    shapes = [np.array(p) + SIGNS * np.array(s) / 2 for p,s,_ in spec.boxes]
    shapes += list(spec.wedges)
    for center, radius, length, axis, _ in spec.cylinders:
        assert axis == "Y"
        shapes.append(np.array(center) + SIGNS * [radius, length/2, radius])
    return np.concatenate(shapes)


POINTS = tuple(vertices(obj) for obj in (BASE, FLAP, PIN))
WITNESS = HingeWitness()
STEP = 0
LAST_STEP = -1
LAST_SCORE = 0.
DWELL = []
CONTACTS = None


@on_reset
def reset(environment):
    global STEP, LAST_STEP, LAST_SCORE, WITNESS, CONTACTS
    STEP, LAST_STEP, LAST_SCORE = 0, -1, 0.
    WITNESS = HingeWitness()
    DWELL.clear()
    path = os.environ.get("HINGE_ARM_CONTACTS")
    if path:
        from hinge_contacts import ArmContacts
        from isaaclab_physx.physics.physx_manager import PhysxManager

        CONTACTS = ArmContacts(path)
        if not getattr(PhysxManager.step, "_a2_contacts", False):
            native_step = PhysxManager.step.__func__

            def step(cls):
                native_step(cls)
                CONTACTS.poll(cls.get_physics_dt())

            step._a2_contacts = True
            PhysxManager.step = classmethod(step)


@on_step
def tick(environment):
    global STEP
    STEP += 1


def world(obj, points):
    return points @ obj.rotation().T + obj.pos


def jaws_clear(geometries):
    base = env().unwrapped
    for side in ("left","right"):
        arm = base.scene[f"{side}_robot"]
        ids, _ = arm.find_joints([*ARM_JOINTS,*FINGER_JOINTS], preserve_order=True)
        q = arm.data.joint_pos.torch[0,ids].detach().cpu().numpy()
        tcp, rotation = ik.tcp_pose(q[:6], station.BASES[side])
        box = tcp + (SIGNS * [max(abs(q[6:]))+.013,.039,.033] + [0.,0.,-.016]) @ rotation.T
        for cloud in geometries:
            if np.maximum(cloud.min(0)-box.max(0), box.min(0)-cloud.max(0)).max() < .010:
                return False
    return True


def steady(previous, current):
    p0, r0 = previous
    pos, rot = current
    return (np.all(np.linalg.norm(pos-p0,axis=1) <= .002)
            and np.all((np.sum(rot*r0,axis=(1,2))-1)/2 >= math.cos(math.radians(2))))


def score(objects):
    global LAST_STEP, LAST_SCORE
    if STEP == LAST_STEP:
        return LAST_SCORE
    base, flap, pin = parts = tuple(objects[name] for name in ("hinge_base","flap","pivot_pin"))
    rotations = np.array([part.rotation() for part in parts])
    positions = np.array([part.pos for part in parts])
    engagement = assess(base.pos,rotations[0],flap.pos,rotations[1],pin.pos,rotations[2])
    angle = flap_angle(rotations[0], rotations[1])
    witnessed = WITNESS.update(engagement, angle)
    current = (positions,rotations)
    if witnessed:
        clouds = [world(part,points) for part,points in zip(parts,POINTS)]
        flap_underside = flap.pos + rotations[1] @ [.045, 0., -.007]
        stop_top = base.pos + rotations[0] @ [.045, 0., -.007]
        on_stop = (abs(angle) < math.radians(6) and abs(clouds[0][:,2].min()-TOP) < .002
                   and abs((flap_underside-stop_top) @ rotations[0][:,2]) < .002)
        if on_stop and jaws_clear(clouds):
            while DWELL and not all(steady(p,current) for p in DWELL):
                DWELL.pop(0)
            DWELL.append(current)
            del DWELL[:-20]
        else:
            DWELL.clear()
    else:
        DWELL.clear()
    LAST_SCORE, LAST_STEP = evaluate(engagement,witnessed,len(DWELL)==20),STEP
    path = os.environ.get("HINGE_DIAGNOSTICS")
    if path:
        row = dict(step=STEP,score=LAST_SCORE,entered=engagement.entered,through_eye=engagement.through_eye,
                   captured=engagement.captured,verified=witnessed,flap_deg=math.degrees(angle),dwell=len(DWELL),
                   positions=positions.tolist(),rotations=rotations.tolist())
        if CONTACTS is not None:
            row["arm_contacts"] = CONTACTS.contacts
        with open(path,"a") as handle:
            handle.write(json.dumps(row)+"\n")
    return LAST_SCORE


def success(objects):
    return score(objects) == 1.


def demo():
    from hinge_pose import as_waypoint_rotation, ready_pose
    left, right = [], []
    open_, close = FINGER_OPEN, FINGER_CLOSED
    lyaw = -math.pi / 2
    ryaw = math.pi / 2
    hover = TOP + .184
    off_left = LEAF_GRIP + [-.003,0.,0.]
    off_right = PIN_GRIP + [-.003,0.,-.003]
    lstart = LEAF_START+off_left
    lseat = AXIS+off_left + [.001,-.001,-.0004]
    rstart = PIN_START+off_right
    rseat = AXIS+off_right + [.0037,.001,.0004]
    rdepth = rseat + [0., .0035, 0.]
    rwait, rywait = ready_pose("right")

    def both(dt, lp, lg, rp, rg, label, lrot=lyaw, rrot=ryaw):
        left.append(Waypoint(dt,tuple(lp),lg,label,lrot))
        right.append(Waypoint(dt,tuple(rp),rg,label,rrot))

    for duration, pos, grip, label in [
        (2.,(*lstart[:2],hover),open_,"left approach"),
        (1.6,lstart,open_,"lower onto the flap handle"),
        (1.,lstart,close,"grasp flap"),
        (1.6,(*lstart[:2],hover),close,"lift flap"),
        (2.,(*lseat[:2],hover),close,"carry flap between ears"),
        (2.8,lseat,close,"align flap eye")]:
        left.append(Waypoint(duration,tuple(pos),grip,label,lyaw))
        right.append(Waypoint(duration,rwait,open_,"right arm parked",rywait))
    both(2.,lseat,close,(*rstart[:2],TOP+.113),open_,"approach pin")
    both(1.6,lseat,close,rstart,open_,"lower onto pin handle")
    both(1.,lseat,close,rstart,close,"grip pin")
    both(1.7,lseat,close,(*rstart[:2],rseat[2]),close,"lift pin to hinge height")
    pin_pitch = .012
    square_pin = np.array([[1., 0., 0.], [0., math.cos(pin_pitch), -math.sin(pin_pitch)],
                           [0., math.sin(pin_pitch), math.cos(pin_pitch)]])
    pin_orientation = as_waypoint_rotation(square_pin @ grip_rotation(ryaw - .040))
    both(2.0,lseat,close,rseat+[0.,-.077,0.],close,"align pin in front of ears",rrot=pin_orientation)
    both(4.4,lseat,close,rseat,close,"insert pin through all three eyes",rrot=pin_orientation)
    both(.6,lseat,close,rdepth,close,"seat pin head",rrot=pin_orientation)
    both(.8,lseat,close,rdepth,open_,"release pin",rrot=pin_orientation)
    both(1.4,lseat,close,(rseat[0],rseat[1]-.075,TOP+.115),open_,"clear pin head")
    theta=math.radians(22)
    tilt=np.array([[math.cos(theta),0.,-math.sin(theta)],[0.,1.,0.],
                   [math.sin(theta),0.,math.cos(theta)]])
    raised=AXIS+tilt@(lseat-AXIS)
    parked=(rseat[0],rseat[1]-.075,TOP+.115)
    both(2.,raised,close,parked,open_,"turn flap around engaged pin",as_waypoint_rotation(tilt@grip_rotation(lyaw)))
    both(2.,lseat,close,parked,open_,"lower flap to its stop")
    both(.8,lseat,open_,parked,open_,"release flap")
    both(1.8,(*lseat[:2],hover),open_,parked,open_,"clear assembly")
    return {"left":left,"right":right}
