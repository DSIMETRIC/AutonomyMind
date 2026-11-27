import json
from pathlib import Path
from .world_model_dynamic import DynamicWorldModel, DynamicTrack
from .dynamic_planning import DynamicPlanningAgent
from .decision import DecisionAgent
from .safety import DeterministicSafetyBridge

class DynamicAvoidancePipeline:
    def __init__(self, trace_path=None):
        self.world=DynamicWorldModel()
        self.planner=DynamicPlanningAgent()
        self.decision=DecisionAgent()
        self.safety=DeterministicSafetyBridge()
        self.trace_path=Path(trace_path) if trace_path else None

    def step(self, frame: dict):
        ego=frame["ego"]
        for t in frame.get("tracks",[]):
            self.world.update_track(DynamicTrack(
                track_id=t["track_id"],label=t.get("label","object"),
                position_m=list(t["position_m"]),velocity_mps=list(t["velocity_mps"]),
                covariance_diag_m2=list(t.get("covariance_diag_m2",[0.1,0.1,0.1])),
                confidence=float(t.get("confidence",1.0)),
                predicted_position_m=list(t.get("predicted_position_m",[])),
            ))
        snapshot=self.world.snapshot(ego,float(frame["timestamp_s"]))
        proposals=self.planner.generate(snapshot,ego.get("robot_id","robot-01"))
        validated=self.safety.validate_candidates(proposals)
        safe=[p for p,d in validated if d.accepted]
        selected=self.decision.select(safe if safe else proposals)
        selected_decision=next((d for p,d in validated if p.proposal_id==selected.evidence[-1].split(":")[-1]),None)
        if selected_decision is None:
            selected_decision=self.safety.validate(selected)
        result={
            "world_state":snapshot,
            "candidates":[{"proposal":p.to_dict(),"safety":d.to_dict()} for p,d in validated],
            "selected_proposal":selected.to_dict(),
            "selected_safety":selected_decision.to_dict(),
        }
        if self.trace_path:
            self.trace_path.parent.mkdir(parents=True,exist_ok=True)
            with self.trace_path.open("a",encoding="utf-8") as f:
                f.write(json.dumps(result,sort_keys=True)+"\n")
        return result
