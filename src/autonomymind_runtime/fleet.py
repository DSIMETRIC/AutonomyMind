"""Transport-neutral fleet coordination reference. No actuator authority."""
from dataclasses import dataclass
import math

def vector(v):
    return len(v)==3 and all(math.isfinite(x) for x in v)

@dataclass(frozen=True)
class Member:
    id:str
    capabilities:frozenset
    position:tuple
    velocity:tuple
    radius_m:float
    uncertainty_m:float
    timestamp:float
    available:bool=True

    def __post_init__(self):
        if not self.id or not vector(self.position) or not vector(self.velocity) or not all(math.isfinite(x) for x in (self.radius_m,self.uncertainty_m,self.timestamp)) or self.radius_m<=0 or self.uncertainty_m<0:
            raise ValueError("invalid member")

@dataclass(frozen=True)
class Task:
    id:str
    required:frozenset
    goal:tuple
    priority:int

    def __post_init__(self):
        if not self.id or not vector(self.goal):raise ValueError("invalid task")

class Fleet:
    def __init__(self,max_age_s):
        if not math.isfinite(max_age_s) or max_age_s<=0:raise ValueError("invalid timeout")
        self.max_age_s=max_age_s;self.members={}
    def update(self,m):
        old=self.members.get(m.id)
        if old is not None and m.timestamp<=old.timestamp:return False
        self.members[m.id]=m;return True
    def fresh(self,m,now):
        return 0<=now-m.timestamp<=self.max_age_s
    def allocate(self,tasks,now):
        if not math.isfinite(now) or len({t.id for t in tasks})!=len(tasks):raise ValueError("invalid task set")
        available={m.id:m for m in self.members.values() if m.available and self.fresh(m,now)}
        assignments={};unassigned=[]
        for t in sorted(tasks,key=lambda x:(-x.priority,x.id)):
            eligible=[m for m in available.values() if t.required<=m.capabilities]
            if not eligible:unassigned.append(t.id);continue
            winner=min(eligible,key=lambda m:(math.dist(m.position,t.goal),m.id))
            assignments[t.id]=winner.id;del available[winner.id]
        return assignments,unassigned
    def conflicts(self,now,horizon_s,margin_m):
        if not all(math.isfinite(x) for x in (now,horizon_s,margin_m)) or min(horizon_s,margin_m)<0:raise ValueError("invalid horizon")
        # Stale peers are explicit unknowns, never silently treated as absent.
        unknown=sorted(m.id for m in self.members.values() if not self.fresh(m,now))
        peers=sorted((m for m in self.members.values() if self.fresh(m,now)),key=lambda m:m.id)
        conflicts=[]
        for i,a in enumerate(peers):
            for b in peers[i+1:]:
                r=[y-x for x,y in zip(a.position,b.position)]
                v=[y-x for x,y in zip(a.velocity,b.velocity)]
                vv=sum(x*x for x in v)
                t=max(0.,min(horizon_s,-sum(x*y for x,y in zip(r,v))/vv)) if vv>1e-12 else 0.
                d=math.sqrt(sum((x+t*y)**2 for x,y in zip(r,v)))
                limit=a.radius_m+b.radius_m+a.uncertainty_m+b.uncertainty_m+margin_m
                if d<=limit:conflicts.append({"a":a.id,"b":b.id,"closest_time_s":t,"separation_m":d,"required_m":limit})
        return {"conflicts":conflicts,"unknown_peers":unknown}
