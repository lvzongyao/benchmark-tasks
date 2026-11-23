"""Nominal Camry pad-and-rotor geometry and two service-gauge profiles."""

import math

STEEL = (.35,.38,.41)
ROTOR = (.58,.63,.67)
FRICTION = (.18,.19,.20)
HAT = (.28,.31,.33)
GREEN = (.23,.62,.32)
RACK = (.25,.36,.40)
RADIUS = .148
ROTOR_HEIGHT = .049
PAD_LENGTH = .158
PAD_WIDTH = .057
EAR_X = .005
EAR_END = -.079
EAR_INNER = -.057
BACKING_THICKNESS = .006
THICKNESSES = (.008,.010)
GRASP = (0.,-.088,.022)
HANDLE_BASE = .004
TOOL_WIDTH = .004


def box(center, size, color):
    return tuple(float(v) for v in center), tuple(float(v) for v in size), color


def ring(inner, outer, lo, hi, n=32):
    profile = ((inner,lo),(outer,lo),(outer,hi),(inner,hi))
    return tuple(
        tuple((r*math.cos(a),r*math.sin(a),z) for a in (i*2*math.pi/n,(i+1)*2*math.pi/n)
              for r,z in profile)
        for i in range(n)
    )


def backing(lining):
    z = lining+BACKING_THICKNESS/2
    return (
        box([0.,0.,z],[.054,.132,BACKING_THICKNESS],STEEL),
        *(box([EAR_X,s*.068,z],[.022,.022,BACKING_THICKNESS],STEEL) for s in (-1,1)),
        box([-.026,0.,z],[.005,.115,BACKING_THICKNESS],STEEL),
        box([.026,0.,z],[.005,.115,BACKING_THICKNESS],STEEL),
        box([0.,0.,lining/2],[.049,.120,lining],FRICTION),
    )


def gauge(thickness):
    lead = thickness/2
    return (
        box([0.,0.,0.],[TOOL_WIDTH,thickness,thickness],GREEN),
        box([0.,-(.046+lead)/2,-lead+.003],[TOOL_WIDTH,.046-lead,.006],GREEN),
        box([0.,-.089,HANDLE_BASE+.009],[TOOL_WIDTH,.029,.018],GREEN),
    )


def gauge_profile(thickness):
    lead = thickness/2
    offset = ((-.046,-lead),(-.074,HANDLE_BASE),(-.074,HANDLE_BASE+.018),(-.046,-lead+.006))
    ramp = tuple((x,y,z) for x in (-TOOL_WIDTH/2,TOOL_WIDTH/2) for y,z in offset)
    # Hollow molded lanyard eye at the heel, outside the grip and measuring faces.
    heel = tuple(tuple((z,y-.103,x+HANDLE_BASE+.009) for y,x,z in segment)
                 for segment in ring(.0032,.009,-TOOL_WIDTH/2,TOOL_WIDTH/2,20))
    return (ramp,*heel)


def gauge_marks(thickness):
    letters = {
        '0': ((0,0),(0,1),(1,1),(1,0),(0,0)),
        '1': ((.5,1),(.5,0)),
        '8': ((1,.5),(1,1),(0,1),(0,0),(1,0),(1,.5),(0,.5)),
        'M': ((0,0),(0,1),(.5,.5),(1,1),(1,0)),
    }
    for i, char in enumerate(f"{round(thickness*1000)}MM"):
        points = letters[char]
        for (a,b),(c,d) in zip(points,points[1:]):
            # Short bars are an approximation of the molded stock markings.
            y = -.098+(i*1.4+(a+c)/2)*.0032
            z = HANDLE_BASE+.005+(b+d)*.0024
            size = (.00009,max(abs(a-c)*.0032,.00036),max(abs(b-d)*.0048,.00036))
            if a == c or b == d:
                yield box([TOOL_WIDTH/2+.00007,y,z],size,(.19,.49,.27))
            else:
                for j in range(6):
                    step = (j+.5)/6-.5
                    yield box([TOOL_WIDTH/2+.00007,y+step*(c-a)*.0032,z+step*(d-b)*.0048],
                              (.00009,.00045,.00045),(.19,.49,.27))
