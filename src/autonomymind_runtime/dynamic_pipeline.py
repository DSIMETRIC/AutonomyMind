import json
import math
from .conflict import closest_approach
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
        # A complete observation frame is required; never reuse stale tracks.
        self.world=DynamicWorldModel()
        ego=frame["ego"]
        def vector(v):
            return isinstance(v,(list,tuple)) and len(v)==3 and all(isinstance(x,(int,float)) and math.isfinite(x) for x in v)
        timestamp=float(frame["timestamp_s"])
        valid=(math.isfinite(timestamp) and vector(ego.get("position_m")) and
               isinstance(frame.get("tracks"),list) and frame.get("data_valid",True) is True and
               all(vector(t.get("position_m")) and vector(t.get("velocity_mps")) for t in frame["tracks"]))
        if not valid:
            return self._record({"world_state":None,"candidates":[],"selected_proposal":None,
                                 "selected_safety":None,"authorization":"denied_invalid_frame"})
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
        validated=self.safety.validate_dynamic_candidates(proposals,snapshot)
        safe=[p for p,d in validated if d.accepted]
        selected=self.decision.select(safe) if safe else None
        selected_decision=self.safety.validate_dynamic(selected,snapshot) if selected else None
        if selected_decision and not selected_decision.accepted:
            selected=None
            selected_decision=None
        result={
            "world_state":snapshot,
            "candidates":[{"proposal":p.to_dict(),"safety":d.to_dict()} for p,d in validated],
            "selected_proposal":selected.to_dict() if selected else None,
            "selected_safety":selected_decision.to_dict() if selected_decision else None,
            "authorization":"reference_accepted" if selected else "denied_no_candidate",
        }
        return self._record(result)

    def _record(self,result):
        if self.trace_path:
            self.trace_path.parent.mkdir(parents=True,exist_ok=True)
            with self.trace_path.open("a",encoding="utf-8") as f:
                f.write(json.dumps(result,sort_keys=True)+"\n")
        return result
