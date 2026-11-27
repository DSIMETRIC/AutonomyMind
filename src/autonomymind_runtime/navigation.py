"""Simulator-neutral 2D reference navigation. Unknown space is not free space."""
from dataclasses import dataclass
import heapq
import math

@dataclass(frozen=True)
class NavigationProfile:
    radius_m: float
    margin_m: float
    speed_mps: float
    max_yaw_rate_rad_s: float
    horizon_s: float
    dt_s: float
    velocity_samples: int
    yaw_samples: int

    def __post_init__(self):
        values=(self.radius_m,self.margin_m,self.speed_mps,self.max_yaw_rate_rad_s,self.horizon_s,self.dt_s)
        if not all(math.isfinite(x) and x>0 for x in values):
            raise ValueError("invalid profile")
        if not 2<=self.velocity_samples<=20 or not 3<=self.yaw_samples<=31 or self.horizon_s/self.dt_s>200 or self.dt_s>self.horizon_s:
            raise ValueError("invalid sampling")

@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float

class OccupancyGrid:
    """Bounded map. Evidence is intentionally simple: occupied dominates free."""
    def __init__(self,width,height,resolution,origin):
        if not 1<=width<=1000 or not 1<=height<=1000 or not math.isfinite(resolution) or resolution<=0:
            raise ValueError("invalid map")
        if len(origin)!=2 or not all(math.isfinite(v) for v in origin):
            raise ValueError("invalid origin")
        self.width=width;self.height=height;self.resolution=resolution;self.origin=tuple(origin)
        self.free=set();self.occupied=set()
    def cell(self,x,y):
        return (math.floor((x-self.origin[0])/self.resolution),math.floor((y-self.origin[1])/self.resolution))
    def center(self,c):
        return tuple(self.origin[i]+(c[i]+.5)*self.resolution for i in range(2))
    def inside(self,c):
        return 0<=c[0]<self.width and 0<=c[1]<self.height
    def observe_ray(self,start,angle,distance,max_range):
        if not all(math.isfinite(v) for v in (*start,angle,max_range)) or max_range<=0 or math.isnan(distance) or distance<=0:
            return
        hit=distance<max_range
        distance=min(distance,max_range)
        if distance/self.resolution>4000:
            raise ValueError("ray too long")
        cells=[]
        for i in range(math.ceil(distance/(self.resolution*.25))+1):
            t=min(distance,i*self.resolution*.25)
            c=self.cell(start[0]+t*math.cos(angle),start[1]+t*math.sin(angle))
            if self.inside(c):cells.append(c)
        endpoint=self.cell(start[0]+distance*math.cos(angle),start[1]+distance*math.sin(angle))
        self.free.update(c for c in cells if c not in self.occupied and (not hit or c!=endpoint))
        if hit and self.inside(endpoint):
            self.occupied.add(endpoint);self.free.discard(endpoint)
    def traversable(self,c,clearance):
        # Conservative circular inflation including unknown and map boundaries.
        n=math.ceil(clearance/self.resolution+math.sqrt(2))
        for dx in range(-n,n+1):
            for dy in range(-n,n+1):
                if math.hypot(max(0,abs(dx)-.5),max(0,abs(dy)-.5))*self.resolution<=clearance:
                    p=(c[0]+dx,c[1]+dy)
                    if not self.inside(p) or p not in self.free or p in self.occupied:return False
        return True

def global_path(grid,start,goal,clearance):
    """Four-connected A*: no diagonal corner cutting, no unknown traversal."""
    a=grid.cell(*start);b=grid.cell(*goal)
    cache={}
    def allowed(c):
        if c not in cache:cache[c]=grid.traversable(c,clearance)
        return cache[c]
    if not allowed(a) or not allowed(b):return []
    queue=[(0,0,a)];cost={a:0};parent={}
    while queue:
        _,g,c=heapq.heappop(queue)
        if g!=cost[c]:continue
        if c==b:
            path=[c]
            while c!=a:c=parent[c];path.append(c)
            return [grid.center(p) for p in reversed(path)]
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            p=(c[0]+dx,c[1]+dy);ng=g+1
            if ng<cost.get(p,math.inf) and allowed(p):
                cost[p]=ng;parent[p]=c
                heapq.heappush(queue,(ng+abs(p[0]-b[0])+abs(p[1]-b[1]),ng,p))
    return []

@dataclass(frozen=True)
class DynamicObstacle:
    x:float
    y:float
    vx:float
    vy:float
    radius_m:float
    uncertainty_m:float

@dataclass(frozen=True)
class Candidate:
    velocity:float
    yaw_rate:float
    points:tuple
    score:float

def local_candidates(grid,pose,target,profile,obstacles=()):
    """Sample constant-control unicycle trajectories. Proposals only; no actuator authority."""
    clearance=profile.radius_m+profile.margin_m
    if not all(math.isfinite(x) for x in (pose.x,pose.y,pose.yaw,*target)):
        return []
    for o in obstacles:
        if not all(math.isfinite(x) for x in (o.x,o.y,o.vx,o.vy,o.radius_m,o.uncertainty_m)) or min(o.radius_m,o.uncertainty_m)<0:
            return []
    result=[]
    dt=min(profile.dt_s,grid.resolution/(4*profile.speed_mps))
    steps=math.ceil(profile.horizon_s/dt)
    if steps>2000:return []
    dt=profile.horizon_s/steps
    for i in range(profile.velocity_samples):
        v=profile.speed_mps*i/(profile.velocity_samples-1)
        for j in range(profile.yaw_samples):
            w=profile.max_yaw_rate_rad_s*(2*j/(profile.yaw_samples-1)-1)
            points=[];safe=True
            for k in range(steps+1):
                t=k*dt
                if abs(w)<1e-9:
                    x=pose.x+v*t*math.cos(pose.yaw);y=pose.y+v*t*math.sin(pose.yaw)
                else:
                    x=pose.x+v/w*(math.sin(pose.yaw+w*t)-math.sin(pose.yaw))
                    y=pose.y+v/w*(math.cos(pose.yaw)-math.cos(pose.yaw+w*t))
                if not grid.traversable(grid.cell(x,y),clearance) or any(
                    math.hypot(x-o.x-o.vx*t,y-o.y-o.vy*t)<=clearance+o.radius_m+o.uncertainty_m for o in obstacles):
                    safe=False;break
                points.append((t,x,y,pose.yaw+w*t))
            if safe:
                _,x,y,yaw=points[-1]
                heading=math.atan2(target[1]-y,target[0]-x)-yaw
                score=math.hypot(target[0]-x,target[1]-y)+.15*abs(math.atan2(math.sin(heading),math.cos(heading)))+.02*abs(w)
                result.append(Candidate(v,w,tuple(points),score))
    return sorted(result,key=lambda c:c.score)

def replan(grid,pose,goal,profile,obstacles=()):
    """Recompute global path and local proposals from current observations."""
    route=global_path(grid,(pose.x,pose.y),goal,profile.radius_m+profile.margin_m)
    if not route:return route,[]
    target=route[min(len(route)-1,max(1,math.ceil(profile.speed_mps*profile.horizon_s/grid.resolution)))]
    return route,local_candidates(grid,pose,target,profile,obstacles)
