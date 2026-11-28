"""Advisory 3D piecewise-linear reservation reference, never a motion gate."""
import math
from dataclasses import dataclass, asdict

@dataclass(frozen=True)
class Request:
    member: str
    priority: int
    radius_m: float
    uncertainty_m: float
    points: tuple  # (relative seconds, x, y, z), first time zero

    def __post_init__(self):
        points=tuple(tuple(p) for p in self.points)
        object.__setattr__(self,"points",points)
        if not isinstance(self.member,str) or not self.member or len(self.member)>128 or not isinstance(self.priority,int):
            raise ValueError("invalid identity or priority")
        if not all(math.isfinite(x) for x in (self.radius_m,self.uncertainty_m)) or self.radius_m<=0 or self.uncertainty_m<0:
            raise ValueError("invalid footprint")
        if not 2<=len(points)<=128 or any(len(p)!=4 or not all(math.isfinite(x) for x in p) for p in points):
            raise ValueError("invalid trajectory")
        if points[0][0]!=0 or any(b[0]<=a[0] for a,b in zip(points,points[1:])):
            raise ValueError("invalid trajectory time")

def segments(request, delay, horizon):
    p=request.points
    if delay<0 or p[-1][0]+delay>horizon:raise ValueError("trajectory outside horizon")
    result=[(a[0]+delay,b[0]+delay,a[1:],b[1:]) for a,b in zip(p,p[1:])]
    if delay>0:result.insert(0,(0.,delay,p[0][1:],p[0][1:]))
    if result[-1][1]<horizon:result.append((result[-1][1],horizon,p[-1][1:],p[-1][1:]))
    return result

def position(segment,t):
    a,b,p,q=segment
    f=(t-a)/(b-a)
    return tuple(x+f*(y-x) for x,y in zip(p,q))

def closest(a,b):
    lo=max(a[0],b[0]);hi=min(a[1],b[1])
    if lo>hi:return None
    r=[x-y for x,y in zip(position(a,lo),position(b,lo))]
    e=[x-y for x,y in zip(position(a,hi),position(b,hi))]
    v=[y-x for x,y in zip(r,e)]
    vv=sum(x*x for x in v)
    f=max(0.,min(1.,-sum(x*y for x,y in zip(r,v))/vv)) if vv>0 else 0.
    return lo+f*(hi-lo),math.hypot(*(x+f*y for x,y in zip(r,v)))

def conflict(a,da,b,db,horizon,margin):
    limit=a.radius_m+b.radius_m+a.uncertainty_m+b.uncertainty_m+margin
    sa=segments(a,da,horizon);sb=segments(b,db,horizon)
    i=j=0;best=None
    while i<len(sa) and j<len(sb):
        c=closest(sa[i],sb[j])
        if c is not None and (best is None or c[1]<best[1]):best=c
        if sa[i][1]<sb[j][1]:i+=1
        else:j+=1
    if best is not None and best[1]<=limit:
        return {"a":a.member,"b":b.member,"closest_time_s":best[0],"separation_m":best[1],"required_m":limit}
    return None

class Coordinator:
    """Atomic whole-fleet advisory revisions. No partial schedule on failure."""
    def __init__(self,members,horizon_s,delay_step_s,max_delay_s,margin_m):
        self.members=frozenset(members)
        if not 1<=len(self.members)<=32 or any(not isinstance(m,str) or not m for m in self.members):
            raise ValueError("invalid fleet")
        if not all(math.isfinite(v) for v in (horizon_s,delay_step_s,max_delay_s,margin_m)) or not 0<horizon_s<=300 or delay_step_s<=0 or not 0<=max_delay_s<=horizon_s or margin_m<0 or max_delay_s/delay_step_s>128:
            raise ValueError("invalid search bounds")
        self.horizon=horizon_s;self.step=delay_step_s;self.max_delay=max_delay_s;self.margin=margin_m
        self.revision=0;self.current=None;self.last_now=-math.inf

    def advance(self,now):
        if not math.isfinite(now) or now<self.last_now:raise ValueError("invalid clock")
        self.last_now=now
        if self.current is not None and now>=self.current["expires_at"]:
            self.current=None;self.revision+=1

    def plan(self,requests,now,expected_revision,lease_s):
        self.advance(now)
        if expected_revision!=self.revision:raise ValueError("stale revision")
        if not math.isfinite(lease_s) or not 0<lease_s<=self.horizon:raise ValueError("invalid lease")
        requests=list(requests)
        if len(requests)!=len(self.members) or {r.member for r in requests}!=self.members:
            raise ValueError("complete fleet proposals required")
        if any(r.points[-1][0]>self.horizon for r in requests):raise ValueError("trajectory outside horizon")
        ordered=sorted(requests,key=lambda r:(-r.priority,r.member))
        initial=[c for i,a in enumerate(ordered) for b in ordered[i+1:] if (c:=conflict(a,0,b,0,self.horizon,self.margin)) is not None]
        selected=[];rejections=[]
        for r in ordered:
            for k in range(math.floor(self.max_delay/self.step)+1):
                delay=k*self.step
                if delay+r.points[-1][0]>self.horizon:break
                blocked=[c for peer,d in selected if (c:=conflict(r,delay,peer,d,self.horizon,self.margin)) is not None]
                if not blocked:
                    selected.append((r,delay));break
                rejections.append({"member":r.member,"delay_s":delay,"conflicts":blocked})
            else:
                pass
            if len(selected)==0 or selected[-1][0].member!=r.member:
                # A failed new revision invalidates the old advisory, never silently reuses it.
                self.current=None;self.revision+=1
                return {"status":"unscheduled","revision":self.revision,"blocked_member":r.member,"initial_conflicts":initial,"rejections":rejections,"reservations":[],"motion_authorized":False,"requires_safety_validation":True}
        self.revision+=1
        result={"status":"scheduled","revision":self.revision,"frame_id":"world_enu","epoch_s":now,"expires_at":now+lease_s,"horizon_s":self.horizon,
                "initial_conflicts":initial,"rejections":rejections,"reservations":[{**asdict(r),"delay_s":d} for r,d in selected],
                "motion_authorized":False,"requires_safety_validation":True}
        self.current=result
        return result
