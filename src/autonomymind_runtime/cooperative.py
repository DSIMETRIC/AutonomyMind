from dataclasses import dataclass, field
import math

@dataclass
class RobotState:
    robot_id: str
    robot_type: str
    position_m: list[float]
    velocity_mps: list[float]
    capabilities: list[str] = field(default_factory=list)
    available: bool = True

@dataclass
class MissionTask:
    task_id: str
    task_type: str
    target_position_m: list[float]
    required_capabilities: list[str]
    priority: int = 50

class SharedWorldModel:
    def __init__(self):
        self.robots={}
        self.tracks={}

    def update_robot(self, robot: RobotState):
        self.robots[robot.robot_id]=robot

    def update_track(self, track: dict):
        self.tracks[track["track_id"]]=dict(track)

    def snapshot(self):
        return {
            "robots":[r.__dict__ for r in self.robots.values()],
            "tracks":list(self.tracks.values()),
        }

class CoordinationAgent:
    def assign(self, tasks: list[MissionTask], robots: list[RobotState]):
        assignments={}
        for task in sorted(tasks,key=lambda t:t.priority,reverse=True):
            candidates=[]
            for r in robots:
                if not r.available:
                    continue
                if any(c not in r.capabilities for c in task.required_capabilities):
                    continue
                d=math.sqrt(sum((r.position_m[i]-task.target_position_m[i])**2 for i in range(3)))
                candidates.append((d,r.robot_id))
            if candidates:
                candidates.sort()
                assignments[task.task_id]=candidates[0][1]
        return assignments

    def inter_robot_conflicts(self, robots: list[RobotState], horizon_s=5.0, min_sep_m=2.0):
        conflicts=[]
        for i,a in enumerate(robots):
            for b in robots[i+1:]:
                rx=[b.position_m[k]-a.position_m[k] for k in range(3)]
                rv=[b.velocity_mps[k]-a.velocity_mps[k] for k in range(3)]
                vv=sum(v*v for v in rv)
                t=0.0 if vv<=1e-12 else max(0.0,min(horizon_s,-sum(rx[k]*rv[k] for k in range(3))/vv))
                sep=math.sqrt(sum((rx[k]+rv[k]*t)**2 for k in range(3)))
                if sep < min_sep_m:
                    conflicts.append({"a":a.robot_id,"b":b.robot_id,"ttc_s":t,"separation_m":sep})
        return conflicts
