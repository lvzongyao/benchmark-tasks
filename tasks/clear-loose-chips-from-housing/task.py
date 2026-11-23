"""Collect ten independent loose chips from an open housing pocket."""

import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent))
from chip_parts import Part, env, on_reset, on_step
from chip_pose import as_waypoint_rotation, ready_pose
from chip_scoring import CHIP_CORNERS, CHIP_SIZE, CUP_FLOOR, CUP_RIM, CUP_X, CUP_Y, SIGNS, captured, score as evaluate
from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint, grip_rotation
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = .05
INSTRUCTION = "Remove all loose packing chips from the housing and collect them in the cup."
TOP = TABLE_TOP[SCENE]
HOME = np.array([.264, .014, TOP])
CUP_HOME = np.array([.264, .186, TOP])
TOOL_HOME = np.array([.287, -.135, TOP])
HOLD = np.array([0., -.052, .057])
HOUSING_COLOR = (.16,.39,.67)
CUP_COLOR = (.19,.58,.38)
TOOL_COLOR = (.89,.48,.17)
CHIP_COLORS = ((.95,.73,.16),(.91,.31,.14))


def xyz(values):
    return tuple(float(value) for value in values)


def box(x0,x1,y0,y1,z0,z1,color):
    lo,hi = np.array([x0,y0,z0]),np.array([x1,y1,z1])
    return xyz((lo+hi)/2),xyz(hi-lo),color


def housing():
    def b(x0,x1,y0,y1,z0,z1):
        return box(x0,x1,y0,y1,z0,z1,HOUSING_COLOR)
    return (
        b(-.045,.045,-.070,.070,0.,.005),
        b(-.045,-.041,-.070,.070,.005,.026),
        b(.041,.045,-.070,.070,.005,.026),
        b(-.041,.041,-.070,-.066,.005,.026),
        b(.030,.035,-.034,.035,.005,.008),
        b(-.014,.014,-.064,-.038,.004,.072),
    )


def catch_cup():
    a,b = CUP_X/2,CUP_Y/2
    return (
        box(-a-.004,a+.004,-b-.004,b+.004,0.,CUP_FLOOR,CUP_COLOR),
        box(-a-.004,-a,-b-.004,b+.004,CUP_FLOOR,CUP_RIM,CUP_COLOR),
        box(a,a+.004,-b-.004,b+.004,CUP_FLOOR,CUP_RIM,CUP_COLOR),
        box(-a,a,-b-.004,-b,CUP_FLOOR,.023,CUP_COLOR),
        box(-a,a,b,b+.004,CUP_FLOOR,CUP_RIM,CUP_COLOR),
    )


HOUSING = Part("housing", pos=xyz(HOME), size=(.090,.140,.072), mass=.16, boxes=housing(), friction=.48)
CUP = Part("catch_cup", pos=xyz(CUP_HOME), size=(CUP_X+.008,CUP_Y+.008,CUP_RIM), mass=.25,
           boxes=catch_cup(), friction=.68)
TOOL = Part("sweeping_tool",pos=xyz(TOOL_HOME+[0.,0.,.003]),size=(.025,.088,.050),mass=.034,friction=.52,
            boxes=(box(-.0125,.0125,-.041,.033,0.,.006,TOOL_COLOR),
                   box(-.012,.012,-.034,-.009,.006,.050,TOOL_COLOR),
                   box(-.0125,.0125,.031,.047,0.,.011,(.76,.71,.50))))
CHIPS = tuple(Part(
    f"chip_{i+1:02d}", pos=xyz(HOME+[x,y,.0072]),size=CHIP_SIZE,mass=.00018,friction=.37,
    boxes=(((0.,0.,0.),CHIP_SIZE,CHIP_COLORS[i%2]),),
) for i,(x,y) in enumerate((x,y) for y in (-.042,-.023,.000,.022,.045) for x in (-.019,.013)))
OBJECTS = [HOUSING,CUP,TOOL,*CHIPS,
           Box("tool_park",pos=xyz(TOOL_HOME+[0.,0.,.0015]),size=(.045,.110,.003),color=(.29,.31,.39))]
HULLS = tuple(np.concatenate([np.array(c)+SIGNS*np.array(s)/2 for c,s,_ in part.boxes])
              for part in (HOUSING,CUP,TOOL))
STEP, LAST_STEP, LAST_SCORE = 0,-1,0.
DWELL = []
INVALID = False
CONTACTS = None


@on_reset
def reset(environment):
    global STEP,LAST_STEP,LAST_SCORE,INVALID,CONTACTS
    STEP,LAST_STEP,LAST_SCORE,INVALID = 0,-1,0.,False
    DWELL.clear()
    path = os.environ.get("CHIPS_ARM_CONTACTS")
    if path:
        from chip_contacts import ArmContacts
        from isaaclab_physx.physics.physx_manager import PhysxManager

        CONTACTS = ArmContacts(path)
        if not getattr(PhysxManager.step,"_d1_contact_trace",False):
            native = PhysxManager.step.__func__
            def step(cls):
                native(cls)
                CONTACTS.poll(cls.get_physics_dt())
            step._d1_contact_trace = True
            PhysxManager.step = classmethod(step)


@on_step
def tick(environment):
    global STEP
    STEP += 1


def clear(clouds):
    arena = env().unwrapped
    for side in ("left","right"):
        robot = arena.scene[f"{side}_robot"]
        ids,_ = robot.find_joints([*ARM_JOINTS,*FINGER_JOINTS],preserve_order=True)
        q = robot.data.joint_pos.torch[0,ids].detach().cpu().numpy()
        tcp,rot = ik.tcp_pose(q[:6],station.BASES[side])
        jaw = tcp+(SIGNS*[max(abs(q[6:]))+.013,.039,.033]+[0.,0.,-.016]) @ rot.T
        for cloud in clouds:
            if np.maximum(cloud.min(0)-jaw.max(0),jaw.min(0)-cloud.max(0)).max() < .010:
                return False
    return True


def score(objects):
    global LAST_STEP,LAST_SCORE,INVALID
    if STEP == LAST_STEP:
        return LAST_SCORE
    housing,cup,tool = rigids = [objects[p.name] for p in (HOUSING,CUP,TOOL)]
    chips = [objects[p.name] for p in CHIPS]
    cup_rot = cup.rotation()
    chips_in_cup = [
        captured((CHIP_CORNERS @ chip.rotation().T + chip.pos-cup.pos) @ cup_rot)
        and np.linalg.norm(chip.lin_vel) < .045 for chip in chips
    ]
    count = sum(bool(p) for p in chips_in_cup)
    positions = np.array([part.pos for part in (*rigids,*chips)])
    INVALID |= bool(not np.isfinite(positions).all() or np.any(positions[:,2] < TOP-.015))
    clouds = [v @ part.rotation().T + part.pos for part,v in zip(rigids,HULLS)]
    tool_parked = (np.linalg.norm(tool.pos[:2]-TOOL_HOME[:2])<=.020 and
                   abs(clouds[2][:,2].min()-(TOP+.003))<=.002)
    floor = all(abs(cloud[:,2].min()-TOP)<=.002 and p.rotation()[2,2]>=.995
                for p,cloud in zip(rigids[:2],clouds[:2]))
    ready = not INVALID and count==10 and tool_parked and floor and clear(clouds)
    if ready:
        while DWELL and any(np.linalg.norm(positions-p,axis=1).max() > .002 for p in DWELL):
            DWELL.pop(0)
        DWELL.append(positions)
        del DWELL[:-20]
    else:
        DWELL.clear()
    LAST_SCORE,LAST_STEP = evaluate(count,not INVALID,len(DWELL)==20),STEP
    path = os.environ.get("CHIPS_DIAGNOSTICS")
    if path:
        row = dict(step=STEP,score=LAST_SCORE,inside=count,valid=not INVALID,dwell=len(DWELL),
                   positions=positions.tolist(),housing_tilt_deg=math.degrees(math.acos(np.clip(housing.rotation()[2,2],-1,1))),
                   arm_contacts=CONTACTS.contacts if CONTACTS is not None else None)
        with open(path,"a") as out:
            out.write(json.dumps(row)+"\n")
    return LAST_SCORE


def success(objects):
    return score(objects)==1.


def demo():
    left,right = [],[]
    park,park_rot = ready_pose("right")
    yaw = -math.pi/2
    straight = grip_rotation(yaw)
    settle = HOME + [0.,0.,.086]
    above_cup = np.array([HOME[0],.122,TOP+.117])

    def add(dt,root,grip,label,angle=0.,offset=(0.,0.,0.)):
        rot = axis_angle_matrix(np.array([1.,0.,0.]),math.radians(angle))
        left.append(Waypoint(dt,xyz(root+rot@HOLD+offset),grip,label,as_waypoint_rotation(rot@straight)))
        right.append(Waypoint(dt,park,FINGER_OPEN,"park tool hand",park_rot))

    add(2.,HOME,FINGER_OPEN,"approach housing handle",offset=(0.,0.,.072))
    add(1.5,HOME,FINGER_OPEN,"lower to handle")
    add(.8,HOME,FINGER_CLOSED,"grip housing")
    add(1.6,settle,FINGER_CLOSED,"lift clear")
    add(2.,above_cup,FINGER_CLOSED,"bring discharge edge over cup")
    for angle in (-15.,-30.,-45.,-60.,-68.):
        add(.65,above_cup,FINGER_CLOSED,"tilt chips into cup",angle)
    add(1.65,above_cup,FINGER_CLOSED,"wait for chips to settle",-68.)
    for angle in (-50.,-34.,-18.,0.):
        add(.48,above_cup,FINGER_CLOSED,"level empty housing",angle)
    add(1.7,settle,FINGER_CLOSED,"return empty housing")
    add(1.5,HOME,FINGER_CLOSED,"place on tabletop")
    add(.75,HOME,FINGER_OPEN,"release")
    add(1.6,HOME,FINGER_OPEN,"retreat",offset=(0.,0.,.096))
    return {"left":left,"right":right}
