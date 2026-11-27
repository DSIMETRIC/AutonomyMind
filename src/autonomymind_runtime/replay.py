import json
from pathlib import Path

def load_trace(path: str):
    events=[]
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if line.strip():
            events.append(json.loads(line))
    return events

def summarize_trace(path: str):
    events=load_trace(path)
    summary={
        "steps":len(events),
        "robots":set(),
        "tracks":set(),
        "assignments":0,
        "conflicts":0,
        "selected_maneuvers":[],
    }
    for e in events:
        coop=e.get("cooperative") or {}
        for r in (coop.get("shared_world") or {}).get("robots",[]):
            summary["robots"].add(r.get("robot_id"))
        for t in (coop.get("shared_world") or {}).get("tracks",[]):
            summary["tracks"].add(t.get("track_id"))
        summary["assignments"] += len(coop.get("assignments",{}))
        summary["conflicts"] += len(coop.get("conflicts",[]))
        dyn=e.get("dynamic_avoidance") or {}
        sp=dyn.get("selected_proposal") or {}
        maneuver=(sp.get("command") or {}).get("maneuver")
        if maneuver:
            summary["selected_maneuvers"].append(maneuver)
    summary["robots"]=sorted(x for x in summary["robots"] if x)
    summary["tracks"]=sorted(x for x in summary["tracks"] if x)
    return summary
