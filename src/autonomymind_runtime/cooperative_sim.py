"""Config-driven kinematic fleet experiment; no physical commands or C++ RTA claim."""
import argparse
import json
import math
from pathlib import Path
from dataclasses import asdict
from .vehicle_models import State,Control,Profile,ModelRegistry
from .navigation import OccupancyGrid,global_path
from .shared_world import SharedWorld
from .dds_protocol import envelope
from .fleet import Member,Task
from .mission_orchestrator import MissionTask,MissionOrchestrator

def xyz(s):return (s.x,s.y,s.z)
def sweep_distance(a,b,c,d):
    r=[x-y for x,y in zip(a,c)]
    v=[(y-x)-(w-z) for x,y,z,w in zip(a,b,c,d)]
    vv=sum(x*x for x in v)
    t=max(0.,min(1.,-sum(x*y for x,y in zip(r,v))/vv)) if vv>1e-12 else 0.
    return math.sqrt(sum((x+t*y)**2 for x,y in zip(r,v)))

def run(s):
    dt=s["dt_s"]; margin=s["margin_m"]; tolerance=s["goal_tolerance_m"]
    if not all(math.isfinite(x) and x>0 for x in (dt,margin,tolerance)) or not 1<=s["steps"]<=5000:raise ValueError("invalid experiment settings")
    raw={v["id"]:v for v in s["vehicles"]}
    if len(raw)!=len(s["vehicles"]) or not 1<=len(raw)<=32:raise ValueError("invalid fleet")
    profiles={k:Profile(**v["profile"]) for k,v in raw.items()}
    if any(p.model not in ("differential","ackermann","omnidirectional","multirotor") for p in profiles.values()):raise ValueError("unsupported experiment model")
    states={k:State(*v["initial"]) for k,v in raw.items()}
    if not all(all(math.isfinite(x) for x in asdict(v).values()) for v in states.values()):raise ValueError("invalid initial states")
    tasks=[MissionTask(Task(t["id"],frozenset(t["required"]),tuple(t["goal"]),t["priority"]),tuple(t.get("dependencies",[]))) for t in s["tasks"]]
    grid=OccupancyGrid(**s["map"])
    # Declared synthetic map prior, not SLAM or sensor-derived free space.
    grid.free={(x,y) for x in range(grid.width) for y in range(grid.height)}
    obstacles=s["obstacles"]
    if len(obstacles)>128:raise ValueError("too many obstacles")
    for o in obstacles:
        if len(o["position"])!=3 or not all(math.isfinite(x) for x in o["position"]) or not math.isfinite(o["radius_m"]) or o["radius_m"]<=0:raise ValueError("invalid obstacle")
    world=SharedWorld(max_age=2*dt)
    mission=MissionOrchestrator(tasks,2*dt,(s["steps"]+2)*dt)
    models=ModelRegistry();trace=[];minimum=None;rejected=0
    velocities={k:(0.,0.,0.) for k in raw}
    for step in range(s["steps"]):
        now=step*dt
        for k in sorted(raw):
            p=profiles[k];v=raw[k];state=states[k]
            payload={"member_id":k,"frame_id":"world_enu","timestamp_s":now,"position_m":list(xyz(state)),"velocity_mps":list(velocities[k]),"uncertainty_m":v["uncertainty_m"],"available":True,"radius_m":p.radius_m,"capabilities":v["capabilities"]}
            if world.ingest(envelope(k,"*","world.member",payload,now,2*dt),now)!="accepted":raise RuntimeError("member evidence rejected")
            # Synthetic range-limited detection; exact simulated object centers, not real perception.
            for i,o in enumerate(obstacles):
                if math.dist(xyz(state),o["position"])<=v["sensor_range_m"]:
                    payload={"observation_id":str(i),"timestamp_s":now,"frame_id":"world_enu","position_m":o["position"],"velocity_mps":[0,0,0],"uncertainty_m":v["uncertainty_m"],"confidence":1.,"label":"synthetic_obstacle","radius_m":o["radius_m"]}
                    status=world.ingest(envelope(k,"*","world.observation",payload,now,2*dt),now)
                    if status!="accepted":raise RuntimeError("observation evidence rejected: "+status)
        snap=world.snapshot(now)
        for record in snap["members"]:
            m=record["payload"]
            mission.update(Member(m["member_id"],frozenset(m["capabilities"]),tuple(m["position_m"]),tuple(m["velocity_mps"]),m["radius_m"],m["uncertainty_m"],m["timestamp_s"],m["available"]),now)
        offers=mission.reconcile(now)
        for k,o in list(offers.items()):
            if math.dist(xyz(states[o.member_id]),mission.tasks[k].task.goal)<=tolerance:mission.complete(k,o.member_id,o.epoch,now)
        offers=mission.reconcile(now)
        if len(mission.completed)==len(tasks):break
        goals={o.member_id:mission.tasks[k].task.goal for k,o in offers.items()}
        observed=[r["payload"] for r in snap["observations"]]
        next_states=dict(states);decisions=[]
        for k in sorted(raw):
            state=states[k];p=profiles[k];goal=goals.get(k)
            if goal is None:continue
            local=OccupancyGrid(grid.width,grid.height,grid.resolution,grid.origin);local.free=set(grid.free)
            clearance=p.radius_m+raw[k]["uncertainty_m"]+margin
            # Project detected spheres near current altitude into a planar route prior.
            for cell in local.free:
                x,y=local.center(cell)
                if any(math.hypot(x-o["position_m"][0],y-o["position_m"][1])<=o["radius_m"]+o["uncertainty_m"]+local.resolution/math.sqrt(2) and abs(state.z-o["position_m"][2])<=clearance+o["radius_m"]+o["uncertainty_m"] for o in observed):
                    local.occupied.add(cell)
            route_clearance=clearance+local.resolution/math.sqrt(2)
            route=global_path(local,(state.x,state.y),goal[:2],route_clearance)
            if not route:
                decisions.append({"member":k,"status":"no_route"});continue
            # Follow the adjacent A* cell, rather than cutting across a corner.
            target=route[1] if len(route)>1 else goal[:2]
            options=[]
            for speed in (0.,p.max_speed_mps/2,p.max_speed_mps):
                for turn in (-p.max_turn_rad,-p.max_turn_rad/2,0.,p.max_turn_rad/2,p.max_turn_rad):
                    verticals=(0.,) if p.model!="multirotor" else (0.,max(-p.max_vertical_mps,min(p.max_vertical_mps,(goal[2]-state.z)/dt)))
                    for vertical in verticals:
                        c=Control(forward=speed,turn=turn,vertical=vertical)
                        samples=[models.propagate(state,c,p,dt*j/8) for j in range(1,9)]
                        end=samples[-1]
                        if any(not local.traversable(local.cell(q.x,q.y),route_clearance) for q in samples):continue
                        if any(math.dist(xyz(q),o["position_m"])<=clearance+o["radius_m"]+o["uncertainty_m"] for q in [state]+samples for o in observed):continue
                        heading=math.atan2(target[1]-end.y,target[0]-end.x)-end.yaw
                        score=math.dist((end.x,end.y,end.z),(*target,goal[2]))+.15*abs(math.atan2(math.sin(heading),math.cos(heading)))
                        options.append((score,c,end,samples))
            accepted=False
            for _,control,end,samples in sorted(options,key=lambda x:x[0]):
                # Independent synthetic truth check for this experiment ONLY.
                # Reserve already selected peer end states; other peers remain stationary.
                # Chord checks include a conservative arc deviation allowance.
                pad=p.max_speed_mps*dt/8
                if any(sweep_distance(xyz(a),xyz(b),o["position"],o["position"])<=p.radius_m+o["radius_m"]+margin+pad for a,b in zip([state]+samples,samples) for o in obstacles):
                    rejected+=1;continue
                if any(sweep_distance(xyz(state),xyz(end),xyz(states[j]),xyz(next_states[j]))<=p.radius_m+profiles[j].radius_m+margin+p.max_speed_mps*dt+profiles[j].max_speed_mps*dt for j in raw if j!=k):
                    rejected+=1;continue
                next_states[k]=end;accepted=True
                decisions.append({"member":k,"status":"kinematic_reference_accepted","control":asdict(control),"route_cells":len(route),"route_target":list(target),"stationary":math.dist(xyz(state),xyz(end))<1e-9 and abs(state.yaw-end.yaw)<1e-9,"candidate_count":len(options),"cpp_rta_assessed":False})
                break
            if not accepted:decisions.append({"member":k,"status":"no_candidate"})
        # Ground truth sampled endpoint separation is a measurement, not continuous collision certification.
        ids=sorted(raw)
        for i,k in enumerate(ids):
            for j in ids[i+1:]:
                sep=math.dist(xyz(next_states[k]),xyz(next_states[j]))-profiles[k].radius_m-profiles[j].radius_m
                minimum=sep if minimum is None else min(minimum,sep)
        velocities={k:tuple((b-a)/dt for a,b in zip(xyz(states[k]),xyz(next_states[k]))) for k in raw}
        states=next_states
        trace.append({"step":step,"t":now+dt,"world_revision":snap["revision"],"shared_observations":len(observed),"assignments":{k:o.member_id for k,o in offers.items()},"states":{k:asdict(v) for k,v in states.items()},"decisions":decisions})
    return {"scenario_id":s["id"],"source":"kinematic_reference_simulation","motion_authorized":False,"cpp_rta_connected":False,
            "metrics":{"tasks_completed":len(mission.completed),"tasks_total":len(tasks),"steps":len(trace),"minimum_sampled_peer_clearance_m":minimum,"reference_rejections":rejected},"mission_events":mission.events,"trace":trace}

def main():
    p=argparse.ArgumentParser();p.add_argument("--scenario",required=True);p.add_argument("--output",required=True);a=p.parse_args()
    result=run(json.loads(Path(a.scenario).read_text()))
    out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    print(json.dumps(result["metrics"],indent=2))
if __name__=="__main__":main()
