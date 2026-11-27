"""Config-driven synthetic event replay; not a physical or closed-loop simulation."""
import argparse
import json
from pathlib import Path
from .fleet import Member, Task
from .mission_orchestrator import MissionTask, MissionOrchestrator

def run(scenario):
    tasks=[MissionTask(Task(t["id"],frozenset(t["required"]),tuple(t["goal"]),t["priority"]),tuple(t.get("dependencies",[]))) for t in scenario["tasks"]]
    config=scenario["coordination"]
    engine=MissionOrchestrator(tasks,config["max_age_s"],config["lease_s"])
    snapshots=[];issued={}
    for event in scenario["events"]:
        now=event["t"]
        for raw in event.get("telemetry",[]):
            m=Member(raw["id"],frozenset(raw["capabilities"]),tuple(raw["position"]),tuple(raw["velocity"]),raw["radius_m"],raw["uncertainty_m"],raw["timestamp"],raw.get("available",True))
            engine.update(m,now)
        # Revoke stale offers before handling completion evidence.
        engine.reconcile(now)
        for o in engine.offers.values():
            issued.setdefault((o.task_id,o.member_id),o.epoch)
        for c in event.get("completions",[]):
            # Synthetic events can refer to the first issued epoch for this pair.
            epoch=c.get("epoch",issued.get((c["task"],c["member"]),-1))
            engine.complete(c["task"],c["member"],epoch,now)
        engine.reconcile(now)
        for o in engine.offers.values():issued.setdefault((o.task_id,o.member_id),o.epoch)
        snapshots.append({"t":now,**engine.snapshot(now,config["horizon_s"],config["margin_m"])})
    events=engine.events
    allocations=[e for e in events if e["kind"]=="offer_issued"]
    reassignments=sum(len({e["member"] for e in allocations if e["task"]==k})>1 for k in engine.tasks)
    return {"scenario_id":scenario["id"],"source":"synthetic_event_replay","physical_hardware":False,
            "motion_authorized":False,"metrics":{"tasks_total":len(tasks),"tasks_completed":len(engine.completed),
            "members_seen":len(engine.fleet.members),"tasks_reassigned_to_different_member":reassignments,
            "revoked_offers":sum(e["kind"]=="offer_revoked" for e in events),
            "rejected_completions":sum(e["kind"]=="completion_rejected" for e in events),
            "conflict_snapshots":sum(bool(s["conflicts"]) for s in snapshots),
            "unknown_peer_snapshots":sum(bool(s["unknown_peers"]) for s in snapshots)},
            "snapshots":snapshots,"events":events}

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--scenario",required=True);parser.add_argument("--output",required=True)
    args=parser.parse_args()
    result=run(json.loads(Path(args.scenario).read_text()))
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    print(json.dumps(result["metrics"],indent=2))
if __name__=="__main__":main()
