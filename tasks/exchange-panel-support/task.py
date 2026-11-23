"""Replace an independent shipping support with a matching foot under a passive hinged panel."""

import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from contacts import Contacts
from partkit import Hinge, env, on_reset, on_step, part
from pose import ready_pose
from scoring import Exchange
from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import Box, TABLE_TOP

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = .05
INSTRUCTION = "Replace the temporary support with the permanent foot while keeping the panel at its current height."
TOP = TABLE_TOP[SCENE]
PIVOT = np.array([.418,.046,TOP+.039])
FRONT = np.array([-.178,.045,-.004])
LEFT_HANDLE = np.array([-.135,.074,.023])
SUPPORT_HEIGHT = .035
WORK = np.array([.254,.028,TOP+.0175])
RIGHT = np.array([.335,-.141,TOP+.0175])
TRAY = np.array([.162,.084,TOP+.0215])
STEM = np.array([0.,-.074,0.])
HOLD = STEM+[0.,0.,.010]
BLUE = (.22,.43,.72)
ORANGE = (.94,.48,.15)
GREEN = (.25,.67,.50)
DARK = (.23,.28,.33)
SCALE = (.91,.87,.69)
SIGNS = np.array([(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)])


def nominal(point):
    return tuple(float(v) for v in point)


PANEL = Hinge("panel", (0.,0.,0.), axis="Y",
              boxes=(((-.09,0.,0.),(.180,.100,.008),BLUE),
                     ((-.135,.056,.003),(.023,.034,.014),BLUE),
                     (nominal(LEFT_HANDLE),(.022,.023,.032),BLUE),
                     ((-.174,.047,-.003),(.010,.007,.002),SCALE)),
              lower=-.30, upper=.25, rate=0., damping=.070, mass=.160, friction=.60)
PANEL_UNIT = part("panel_unit",nominal(PIVOT),
                  boxes=(((.038,0.,-.034),(.120,.118,.010),DARK),
                         ((-.002,.051,-.017),(.021,.009,.033),DARK),
                         ((-.002,-.051,-.017),(.021,.009,.033),DARK)),
                  mass=.800, hinges=(PANEL,),friction=.86)


def support(name, pos, color):
    return part(name,nominal(pos),
                boxes=(((0.,0.,0.),(.038,.028,SUPPORT_HEIGHT),color),
                       ((0.,-.043,-.006),(.018,.074,.014),color),
                       (nominal(STEM),(.024,.024,.035),color)),
                mass=.080 if name=="temporary" else .103, friction=.64)


OLD = support("temporary",WORK,ORANGE)
NEW = support("foot",RIGHT,GREEN)
OBJECTS = [PANEL_UNIT,OLD,NEW,
           Box("slot_front",pos=(.224,.026,TOP+.003),size=(.009,.064,.006),color=DARK),
           Box("slot_rear",pos=(.284,.026,TOP+.003),size=(.009,.064,.006),color=DARK),
           Box("slot_end",pos=(.254,.0465,TOP+.003),size=(.048,.007,.006),color=DARK),
           Box("tray_floor",pos=(.162,.051,TOP+.002),size=(.084,.137,.004),color=DARK),
           Box("tray_edge_front",pos=(.1175,.051,TOP+.006),size=(.005,.137,.012),color=DARK),
           Box("tray_edge_rear",pos=(.2065,.051,TOP+.006),size=(.005,.137,.012),color=DARK),
           Box("scale_base",pos=(.248,.112,TOP+.021),size=(.012,.014,.042),color=DARK)]
for index, rise in enumerate((-.009,-.006,-.003,0.,.003,.006)):
    OBJECTS.append(Box(f"scale_{index}",pos=(.2415,.112,TOP+.035+rise),
                   size=(.001,.013,.001),color=ORANGE if rise==0. else SCALE))

STATE = {}


def pitch(angle):
    c,s = math.cos(angle), math.sin(angle)
    return np.array([[c,0.,s],[0.,1.,0.],[-s,0.,c]])


@on_reset
def reset(_):
    from isaaclab_physx.physics.physx_manager import PhysxManager

    STATE.update(tick=0, trial=Exchange(), contact=Contacts(), history=[], height=None)
    if not getattr(PhysxManager.step, "_d5_native", False):
        native = PhysxManager.step.__func__

        def step(cls):
            native(cls)
            STATE["contact"].poll(cls.get_physics_dt())

        step._d5_native = True
        PhysxManager.step = classmethod(step)


@on_step
def tick(_):
    STATE["tick"] += 1


def vertices(spec):
    return np.concatenate([np.array(c)+SIGNS*np.array(s)/2 for c,s,_ in spec.boxes])


SHAPES = [vertices(spec) for spec in (PANEL,OLD,NEW)]


def hands_clear(clouds):
    env_scene = env().unwrapped.scene
    for side in ("left","right"):
        arm = env_scene[f"{side}_robot"]
        ids,_ = arm.find_joints([*ARM_JOINTS,*FINGER_JOINTS],preserve_order=True)
        q = arm.data.joint_pos.torch[0,ids].cpu().numpy()
        p,r = ik.tcp_pose(q[:6],station.BASES[side])
        hand = p+(SIGNS*[max(abs(q[6:]))+.013,.039,.033]+[0.,0.,-.016]) @ r.T
        if any(np.maximum(cloud.min(0)-hand.max(0),hand.min(0)-cloud.max(0)).max() < .010 for cloud in clouds):
            return False
    return True


def score(objects):
    trial,tick,contact = (STATE[n] for n in ("trial","tick","contact"))
    if tick == trial.last_tick:
        return trial.score
    root,old,foot = (objects[n] for n in ("panel_unit","temporary","foot"))
    rotation = root.rotation() @ pitch(root.hinges["panel"])
    position = np.array([root.pos,old.pos,foot.pos])
    rotations = np.array([rotation,old.rotation(),foot.rotation()])
    world = [cloud @ r.T + p for cloud,r,p in zip(SHAPES,rotations,position)]
    height = float((root.pos+rotation @ FRONT)[2])
    if STATE["height"] is None:
        STATE["height"] = height
    err = height-STATE["height"]
    withdrawn = max(0., float(WORK[1]-old.pos[1]))
    old_clear = bool(abs(old.pos[1]-WORK[1]) > .053 or abs(old.pos[0]-WORK[0]) > .064)
    staged = old_clear and bool(np.linalg.norm(old.pos[:2]-np.array([.180,-.087])) < .055
                               or np.linalg.norm(old.pos[:2]-TRAY[:2]) < .042)
    in_tray = (old_clear and np.linalg.norm(old.pos[:2]-TRAY[:2]) < .016
               and abs(old.pos[2]-TRAY[2]) < .003 and rotations[1][2,2] > .99)
    temporary_n,foot_n = contact.snapshot["support"]
    robot_n = min(p[0] for p in contact.snapshot["panel_grip"])
    supported = temporary_n > .18 or foot_n > .18 or robot_n > .25
    foot_error = float(np.linalg.norm(foot.pos[:2]-WORK[:2]))
    new_support = old_clear and foot_n > .18 and foot_error <= .003 and abs(err) <= .002
    history = STATE["history"]
    if in_tray and new_support and hands_clear(world):
        while history and any(np.max(np.linalg.norm(position-p,axis=1)) > .002
                              or np.min((np.sum(rotations*r,axis=(1,2))-1)/2) < math.cos(math.radians(2))
                              for p,r in history):
            history.pop(0)
        history.append((position,rotations))
        del history[:-20]
    else:
        history.clear()
    valid = bool(np.all(position[:,2] > TOP-.040) and np.linalg.norm(root.pos-PIVOT) < .003)
    if old_clear:
        withdrawn = max(withdrawn, float(np.linalg.norm(old.pos[:2]-WORK[:2])))
    value = trial.update(tick,err,withdrawn,supported,staged,new_support,len(history)>=20,valid)
    path = os.environ.get("SUPPORT_DIAGNOSTICS")
    if path:
        with open(path,"a") as stream:
            stream.write(json.dumps(dict(step=tick,score=value,front_height_mm=(height-TOP)*1000,
                height_error_mm=err*1000,extent_mm=[trial.minimum*1000,trial.maximum*1000],
                withdrawn_mm=withdrawn*1000,old_clear=old_clear,old_staged=staged,in_tray=bool(in_tray),
                foot_error_mm=foot_error*1000,new_support=bool(new_support),old_n=temporary_n,foot_n=foot_n,
                grip_n=robot_n,right_finger_contacts=contact.snapshot["block_grip"],invalid=trial.invalid,dwell=len(history),positions=position.tolist(),
                angle_deg=math.degrees(root.hinges["panel"]),arm_contacts=int(contact.cross_count),
                contact_samples=contact.sample_count))+"\n")
    return value


def success(objects):
    return score(objects) == 1.


def demo():
    left,right = [],[]
    lyaw,ryaw = 0.,math.pi/2
    lhold = PIVOT+LEFT_HANDLE+[0.,-.001,.004]
    llift = lhold+[0.,0.,.0025]
    lpark = lhold+[0.,.055,.050]
    rstart = WORK+HOLD
    rstop = rstart+[0.,-.102,.001]
    rtray = TRAY+HOLD
    rfoot = RIGHT+HOLD
    rent = WORK+HOLD+[.0043,-.086,.001]
    rplace = WORK+HOLD+[.0043,-.0005,.001]
    h = np.array([0.,0.,.077])
    opened,closed = FINGER_OPEN,FINGER_CLOSED
    idle = os.environ.get("SUPPORT_CONTROL") == "unsupported"
    rwait,rr_wait = ready_pose("right")
    lwait,lr_wait = ready_pose("left")

    def both(dt, lp, lg, rp, rg, label, rr=ryaw):
        left.append(Waypoint(dt,tuple(lwait if idle else lp),opened if idle else lg,label,lr_wait if idle else lyaw))
        right.append(Waypoint(dt,tuple(rp),rg,label,rr))

    both(2.0,lhold+h,opened,rwait,opened,"approach panel grasp",rr_wait)
    both(1.4,lhold,opened,rwait,opened,"descend to panel grasp",rr_wait)
    both(.9,lhold,closed,rwait,opened,"grasp panel",rr_wait)
    both(.9,llift,closed,rwait,opened,"take the panel load",rr_wait)
    both(2.0,llift,closed,rstart+h,opened,"approach temporary support handle")
    both(1.3,llift,closed,rstart,opened,"descend to temporary support")
    both(.9,llift,closed,rstart,closed,"grasp shipping block")
    both(2.8,llift,closed,rstop,closed,"extract temporary support")
    around = np.array([.130,rstop[1],rstop[2]+.059])
    both(1.8,llift,closed,around,closed,"carry temporary support beyond panel")
    both(2.6,llift,closed,rtray+h,closed,"translate support over its tray")
    both(1.4,llift,closed,rtray,closed,"place temporary support into tray")
    both(.9,llift,closed,rtray,opened,"release temporary support")
    both(1.5,llift,closed,rtray+h,opened,"clear shipping support")
    both(2.5,llift,closed,rfoot+h,opened,"approach permanent foot")
    both(1.4,llift,closed,rfoot,opened,"descend onto permanent foot")
    both(.9,llift,closed,rfoot,closed,"grasp permanent foot")
    both(1.4,llift,closed,rfoot+[0.,0.,.033],closed,"lift permanent foot")
    both(1.6,llift,closed,rent,closed,"lower into locating channel")
    both(2.8,llift,closed,rplace,closed,"slide permanent foot into recess")
    both(1.3,lhold,closed,rplace,closed,"transfer the panel load")
    both(1.1,lhold,opened,rplace,opened,"release panel and permanent foot")
    both(2.0,lpark,opened,rplace+[0.,-.037,.060],opened,"retreat from the panel and foot")
    return {"left":left,"right":right}
