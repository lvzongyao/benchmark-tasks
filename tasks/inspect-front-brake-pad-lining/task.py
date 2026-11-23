"""Check a removed Camry front pad's friction lining with 8 and 10 mm Lisle gauges."""

import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from contacts import Contacts
import geometry as geo
from partkit import Part, env, on_reset, on_step
from pose import as_waypoint_rotation, ready_pose
from scoring import Measurement
from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint, grip_rotation
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = .05
INSTRUCTION = "Check the brake pad lining with both gauges, then return them to their rests."
TOP = TABLE_TOP[SCENE]
FACE = TOP+geo.ROTOR_HEIGHT
ROTOR = np.array([.188,.045,TOP])
PAD = np.array([.291,.046,FACE])
RESTS = np.array([[.328,-.145,TOP+.018],[.215,-.145,TOP+.018]])
LINING = {"narrow":.0066,"wide":.0114}.get(os.environ.get("GAUGE_CONTROL_GAP"),.0084)
NAMES = ("gauge_8mm","gauge_10mm")
CORNERS = np.array([(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)])
STATE = {}


def xyz(point):
    return tuple(float(v) for v in point)


def rack(name, at, thickness):
    surface = at[2]+geo.HANDLE_BASE
    floor = Box(name+"_floor",pos=(float(at[0]),float(at[1]-.094),float((surface+TOP)/2)),
                size=(.032,.038,float(surface-TOP)),color=geo.RACK)
    rails = [
        Box(f"{name}_rail_{i}",pos=xyz(at+[s*.007,-.094,geo.HANDLE_BASE+.002]),
            size=(.004,.038,.004),color=geo.RACK) for i,s in enumerate((-1,1))
    ]
    probe_z = float(at[2]-thickness/2)
    probe = Box(name+"_probe_rest",pos=(float(at[0]),float(at[1]-.023),(probe_z+TOP)/2),
                size=(.017,.016,probe_z-TOP),color=geo.RACK)
    return [floor,*rails,probe]


rotor = Part("rotor",pos=xyz(ROTOR),size=(.296,.296,.049),mass=7.2,color=geo.ROTOR,friction=.72,
              wedges=geo.ring(.071,geo.RADIUS,.021,geo.ROTOR_HEIGHT)+geo.ring(.031,.076,0.,.030,32))
pad = Part("pad",pos=xyz(PAD),size=(geo.PAD_WIDTH,geo.PAD_LENGTH,.021),mass=.46,
           boxes=geo.backing(LINING),color=geo.STEEL,friction=.74)
gauges = [Part(name,pos=xyz(at),size=(.004,.117,.027),mass=.009,boxes=geo.gauge(thickness),
               wedges=geo.gauge_profile(thickness),marks=tuple(geo.gauge_marks(thickness)),
               color=geo.GREEN,friction=.62)
          for name,at,thickness in zip(NAMES,RESTS,geo.THICKNESSES)]
OBJECTS = [rotor,pad,*gauges,
           *(p for name,at,thickness in zip(NAMES,RESTS,geo.THICKNESSES) for p in rack(name,at,thickness))]


@on_reset
def reset(_):
    from isaaclab_physx.physics.physx_manager import PhysxManager

    STATE.update(tick=0,measurement=Measurement(),contact=Contacts(NAMES),start=None,parking=0)
    if not getattr(PhysxManager.step,"_d4_lining",False):
        native = PhysxManager.step.__func__

        def step(cls):
            native(cls)
            STATE["contact"].poll(cls.get_physics_dt())

        step._d4_lining = True
        PhysxManager.step = classmethod(step)


@on_step
def tick(_):
    STATE["tick"] += 1


def hand_clear(points):
    scene = env().unwrapped.scene
    for side in ("left","right"):
        arm = scene[f"{side}_robot"]
        ids,_ = arm.find_joints([*ARM_JOINTS,*FINGER_JOINTS],preserve_order=True)
        q = arm.data.joint_pos.torch[0,ids].cpu().numpy()
        point,rotation = ik.tcp_pose(q[:6],station.BASES[side])
        box = point+(CORNERS*[max(abs(q[6:]))+.013,.039,.033]+[0.,0.,-.016]) @ rotation.T
        if any(np.maximum(p.min(0)-box.max(0),box.min(0)-p.max(0)).max() < .010 for p in points):
            return False
    return True


def rotation_error(r):
    return math.degrees(math.acos(np.clip((np.trace(r)-1)/2,-1.,1.)))


def forces(tool_index, tool, pad_state, rotor_state, contacts):
    pr = pad_state.rotation()
    rr = rotor_state.rotation()
    tr = tool.rotation()
    half = geo.THICKNESSES[tool_index]/2
    face, backing, stop = 0.,0.,0.
    for force, point, normal in contacts.pairs[tool_index][0]:
        if abs((point-tool.pos) @ tr[:,1]) > half+.001:
            continue
        x,y,z = (point-pad_state.pos) @ pr
        zrotor = ((point-rotor_state.pos) @ rr)[2]
        if abs(x-geo.EAR_X) <= .012 and geo.EAR_END-.013 <= y <= geo.EAR_INNER:
            if abs(zrotor-geo.ROTOR_HEIGHT) < .0012 and abs(normal @ rr[:,2]) > .75:
                face += abs(float(force))
    for force, point, normal in contacts.pairs[tool_index][1]:
        if abs((point-tool.pos) @ tr[:,1]) > half+.001:
            continue
        x,y,z = (point-pad_state.pos) @ pr
        if abs(x-geo.EAR_X) <= .012 and geo.EAR_END-.0012 <= y <= geo.EAR_INNER:
            if abs(z-LINING) < .0012 and abs(normal @ pr[:,2]) > .75:
                backing += abs(float(force))
            if abs(y-geo.EAR_END) < .0013 and LINING-.001 <= z <= LINING+.004:
                if abs(normal @ pr[:,1]) > .70:
                    stop += abs(float(force))
    return face,backing,stop


def score(objects):
    count,measurement,contact = (STATE[k] for k in ("tick","measurement","contact"))
    if count == measurement.last_tick:
        return measurement.score
    rotor_state,pad_state,*tools = [objects[name] for name in ("rotor","pad",*NAMES)]
    relative = (pad_state.pos-rotor_state.pos) @ rotor_state.rotation()
    axis = rotor_state.rotation().T @ pad_state.rotation()
    if STATE["start"] is None:
        STATE["start"] = (relative.copy(),axis.copy(),rotor_state.pos.copy(),rotor_state.rotation())
    start,heading,rotor_origin,rotor_heading = STATE["start"]
    drift = float(np.linalg.norm(relative-start))
    pad_angle = rotation_error(heading.T @ axis)
    rotor_drift = float(np.linalg.norm(rotor_state.pos-rotor_origin))
    rotor_angle = rotation_error(rotor_heading.T @ rotor_state.rotation())
    intact = all(part.pos[2] >= TOP-.022 for part in (pad_state,*tools))
    tip = [(tool.pos-pad_state.pos) @ pad_state.rotation() for tool in tools]
    angles = [rotation_error(pad_state.rotation().T @ tool.rotation()) for tool in tools]
    interface = [forces(i,tool,pad_state,rotor_state,contact) for i,tool in enumerate(tools)]
    lining_at_ear = relative+axis @ [geo.EAR_X,geo.EAR_END,0.]
    face_gap = float(lining_at_ear[2])-geo.ROTOR_HEIGHT
    physically_supported = (contact.pad_support > 1. and abs(face_gap) <= .00065
                            and axis[2,2] >= math.cos(math.radians(2.)))
    floor,roof,_ = interface[0]
    go_depth = tip[0][1]+geo.THICKNESSES[0]/2-geo.EAR_END
    go = .007 <= go_depth <= .021 and angles[0] <= 5. and physically_supported
    nogo_depth = tip[1][1]+geo.THICKNESSES[1]/2-geo.EAR_END
    nogo = abs(nogo_depth) <= .002 and angles[1] <= 5. and physically_supported
    blocked = nogo and interface[1][0] > .035 and interface[1][2] > .035
    parked = all(np.linalg.norm((tool.pos-at)[:2]) <= .009
                 and abs(tool.pos[2]-at[2]) <= .003 and tool.rotation()[2,2] > .994
                 and np.linalg.norm(tool.lin_vel) < .003 and np.linalg.norm(tool.ang_vel) < .12
                 for tool,at in zip(tools,RESTS))
    parked = parked and physically_supported
    if parked:
        areas = [tool.pos+CORNERS*[.005,.060,.015]+[0.,-.053,.008] for tool in tools]
        parked = hand_clear(areas+[pad_state.pos+CORNERS*[.029,.079,.010]+[0.,0.,.010]])
    STATE["parking"] = STATE["parking"]+1 if parked else 0
    result = measurement.update(count,go and floor > .035,go and roof > .035,blocked,
                                intact,STATE["parking"] >= 20)
    path = os.environ.get("GAUGE_DIAGNOSTICS")
    if path:
        with open(path,"a") as file:
            file.write(json.dumps(dict(step=count,score=result,tip_mm=[(v*1000).tolist() for v in tip],
                depth_mm=[float(go_depth*1000),float(nogo_depth*1000)],angles=angles,contacts_n=interface,
                pad_support_n=contact.pad_support,pad_drift_mm=drift*1000,pad_angle=pad_angle,
                rotor_drift_mm=rotor_drift*1000,rotor_angle=rotor_angle,
                witness=measurement.witness,samples=measurement.counts,intact=bool(intact),
                lining_face_gap_mm=face_gap*1000,supported=bool(physically_supported),
                parked=bool(parked),positions=[tool.pos.tolist() for tool in tools],
                tool_rotation=[tool.rotation().tolist() for tool in tools],
                raw_contacts=[[sum(abs(float(force)) for force,_,_ in pair) for pair in tool_pairs]
                              for tool_pairs in contact.pairs],
                rotor_contact_axial_mm=[
                    [float((point-tool.pos) @ tool.rotation()[:,1])*1000 for _,point,_ in pairs[0]]
                    for tool,pairs in zip(tools,contact.pairs)
                ],
                arm_contacts=int(contact.arm_collisions)))+"\n")
    return result


def success(objects):
    return score(objects) == 1.


def demo():
    plan = []
    yaw = math.pi/2
    ready,rotation = ready_pose("right")
    handle = np.array(geo.GRASP)
    nose_down = axis_angle_matrix(np.array([1.,0.,0.]),math.radians(-1.2))
    feed_rotation = as_waypoint_rotation(nose_down @ grip_rotation(yaw))
    offset = nose_down @ handle-handle
    entry = np.array([PAD[0]+geo.EAR_X,PAD[1]+geo.EAR_END-.013,FACE])
    clear = np.array([0.,0.,.055])
    opened,closed = FINGER_OPEN,FINGER_CLOSED

    def put(seconds, point, grip, label, angle=yaw):
        plan.append(Waypoint(seconds,xyz(point),grip,label,angle))

    for index,(rest,t) in enumerate(zip(RESTS,geo.THICKNESSES)):
        pick = rest+handle
        park = pick+[.006,0.,0.]
        contact = entry+handle+[0.,0.,t/2-.0010]
        stroke = .023 if index == 0 else .013-t/2+.0018
        target = contact+[0.,stroke,0.]
        put(1.35,pick+clear,opened,"approach the gauge")
        put(1.15,pick,opened,"lower over its stock handle")
        put(.85,pick,closed,"grasp the stock handle")
        put(1.2,pick+clear,closed,"lift the gauge from its rest")
        put(1.55,contact+offset+[0.,0.,.035],closed,"approach the pad ear",feed_rotation)
        put(1.35,contact+offset,closed,"align the probe with the rotor face",feed_rotation)
        put(2.1,target+offset,closed,"feed the gauge beside the friction lining",feed_rotation)
        put(.85,target+offset,closed,"hold the rotor contact",feed_rotation)
        if index == 0:
            put(.85,target+[0.,0.,.0029],closed,"feel the backing with the fitted gauge")
            put(.95,target+[0.,0.,.0029],closed,"hold the backing contact")
            put(.7,target+[0.,0.,.0008],closed,"relax the test pressure")
        withdrawal_offset = offset if index else 0.
        withdrawal_angle = feed_rotation if index else yaw
        put(1.6,contact+withdrawal_offset+[0.,0.,.0010],closed,"withdraw the gauge",withdrawal_angle)
        put(1.2,contact+[0.,0.,.040],closed,"lift clear of the rotor")
        put(1.4,park+clear,closed,"return above its rest")
        put(1.2,park,closed,"seat the gauge in its rest")
        put(.75,park,opened,"release the gauge")
        put(1.15,park+clear,opened,"retreat from the parked gauge")
    put(1.45,ready,opened,"clear both gauges",rotation)
    return {"right":plan}
