"""Reference kinematics, independent of simulator and actuator transport."""
from dataclasses import dataclass
import math
import json

@dataclass(frozen=True)
class State:
    x: float
    y: float
    z: float
    yaw: float

@dataclass(frozen=True)
class Control:
    forward: float = 0.
    lateral: float = 0.
    vertical: float = 0.
    turn: float = 0.

@dataclass(frozen=True)
class Profile:
    model: str
    radius_m: float
    max_speed_mps: float
    max_turn_rad: float
    max_vertical_mps: float = 0.
    wheelbase_m: float = 0.
    min_speed_mps: float = 0.

    def __post_init__(self):
        a=(self.radius_m,self.max_speed_mps,self.max_turn_rad,self.max_vertical_mps,self.wheelbase_m,self.min_speed_mps)
        if not all(math.isfinite(x) and x>=0 for x in a) or min(a[:3])<=0 or self.min_speed_mps>self.max_speed_mps:
            raise ValueError("invalid limits")
        if self.model=="ackermann" and (self.wheelbase_m<=0 or self.max_turn_rad>=math.pi/2):
            raise ValueError("invalid steering geometry")
        if self.model=="fixed_wing" and self.min_speed_mps<=0:
            raise ValueError("fixed wing requires nonzero minimum speed")

def load_profile(path):
    with open(path,encoding="utf-8") as f:
        return Profile(**json.load(f))

def finite_state(s):
    return all(math.isfinite(x) for x in (s.x,s.y,s.z,s.yaw))

def _arc(s,v,w,dt):
    if abs(w)<1e-10:
        return State(s.x+v*dt*math.cos(s.yaw),s.y+v*dt*math.sin(s.yaw),s.z,s.yaw)
    return State(s.x+v/w*(math.sin(s.yaw+w*dt)-math.sin(s.yaw)),s.y+v/w*(math.cos(s.yaw)-math.cos(s.yaw+w*dt)),s.z,s.yaw+w*dt)

def differential(s,c,p,dt):
    if c.lateral or c.vertical:raise ValueError("unsupported axes")
    return _arc(s,c.forward,c.turn,dt)

def ackermann(s,c,p,dt):
    if c.lateral or c.vertical:raise ValueError("unsupported axes")
    return _arc(s,c.forward,c.forward*math.tan(c.turn)/p.wheelbase_m,dt)

def omnidirectional(s,c,p,dt):
    if c.vertical:raise ValueError("unsupported axis")
    # Integrate body-frame translation under constant yaw rate exactly.
    a=_arc(s,c.forward,c.turn,dt)
    b=_arc(State(s.x,s.y,s.z,s.yaw+math.pi/2),c.lateral,c.turn,dt)
    return State(a.x+b.x-s.x,a.y+b.y-s.y,s.z,a.yaw)

def multirotor(s,c,p,dt):
    a=omnidirectional(s,Control(c.forward,c.lateral,0.,c.turn),p,dt)
    return State(a.x,a.y,s.z+c.vertical*dt,a.yaw)

def fixed_wing(s,c,p,dt):
    if c.lateral:raise ValueError("unsupported lateral command")
    a=_arc(s,c.forward,c.turn,dt)
    return State(a.x,a.y,s.z+c.vertical*dt,a.yaw)

class ModelRegistry:
    def __init__(self):
        self.models={"differential":differential,"ackermann":ackermann,"omnidirectional":omnidirectional,"multirotor":multirotor,"fixed_wing":fixed_wing}
    def register(self,name,model):
        if not name or name in self.models or not callable(model):raise ValueError("invalid model")
        self.models[name]=model
    def propagate(self,state,control,profile,dt):
        c=control;p=profile
        if not finite_state(state) or not all(math.isfinite(v) for v in (c.forward,c.lateral,c.vertical,c.turn,dt)) or dt<=0:
            raise ValueError("invalid state/control")
        if math.hypot(c.forward,c.lateral)>p.max_speed_mps or abs(c.turn)>p.max_turn_rad or abs(c.vertical)>p.max_vertical_mps or c.forward<p.min_speed_mps:
            raise ValueError("control outside profile")
        if p.model not in self.models:raise ValueError("unknown model")
        result=self.models[p.model](state,c,p,dt)
        if not finite_state(result):raise ValueError("invalid propagation")
        return result
