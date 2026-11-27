import json
from pathlib import Path
from .cooperative_pipeline import CooperativePipeline
from .dynamic_pipeline import DynamicAvoidancePipeline

def run_exchange_jsonl(input_path: str, trace_path: str):
    src=Path(input_path)
    frames=[json.loads(line) for line in src.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not frames:
        raise ValueError("exchange stream contains no frames")
    coop=CooperativePipeline()
    dynamic=DynamicAvoidancePipeline()
    trace=Path(trace_path)
    trace.parent.mkdir(parents=True,exist_ok=True)
    trace.write_text("",encoding="utf-8")
    count=0
    for frame in frames:
        coop_result=coop.step({
            "robots":frame.get("robots",[]),
            "tasks":frame.get("tasks",[]),
            "tracks":frame.get("tracks",[]),
        })
        rover=next((r for r in frame.get("robots",[]) if r.get("robot_type")=="rover"),None)
        dyn=None
        if rover:
            dyn=dynamic.step({
                "timestamp_s":frame.get("timestamp_s",0.0),
                "ego":{
                    "robot_id":rover["robot_id"],
                    "position_m":rover.get("position_m",[0,0,0]),
                    "speed_mps":abs(float(rover.get("velocity_mps",[0,0,0])[0])),
                },
                "tracks":frame.get("tracks",[]),
            })
        event={
            "event_type":"mission_step",
            "step":count,
            "timestamp_s":float(frame.get("timestamp_s",0.0)),
            "cooperative":coop_result,
            "dynamic_avoidance":dyn,
            "source":"isaac_exchange",
        }
        with trace.open("a",encoding="utf-8") as f:
            f.write(json.dumps(event,sort_keys=True)+"\n")
        count += 1
    return count
