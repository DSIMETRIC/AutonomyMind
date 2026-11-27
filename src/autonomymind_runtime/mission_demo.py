import json
from pathlib import Path
from .cooperative_pipeline import CooperativePipeline
from .dynamic_pipeline import DynamicAvoidancePipeline

def run_mission(scenario_path: str, trace_path: str):
    data=json.loads(Path(scenario_path).read_text(encoding="utf-8"))
    frames=data.get("frames",[])
    if not frames:
        raise ValueError("mission scenario contains no frames")

    coop=CooperativePipeline()
    dynamic=DynamicAvoidancePipeline()
    trace=Path(trace_path)
    trace.parent.mkdir(parents=True,exist_ok=True)
    trace.write_text("",encoding="utf-8")

    results=[]
    for index,frame in enumerate(frames):
        coop_result=coop.step({
            "robots":frame.get("robots",[]),
            "tasks":frame.get("tasks",[]),
            "tracks":frame.get("tracks",[]),
        })
        rover=next((r for r in frame.get("robots",[]) if r.get("robot_type")=="rover"),None)
        dynamic_result=None
        if rover:
            dynamic_result=dynamic.step({
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
            "step":index,
            "timestamp_s":float(frame.get("timestamp_s",0.0)),
            "cooperative":coop_result,
            "dynamic_avoidance":dynamic_result,
        }
        with trace.open("a",encoding="utf-8") as f:
            f.write(json.dumps(event,sort_keys=True)+"\n")
        results.append(event)
    return results
