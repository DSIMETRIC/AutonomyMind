from .contracts import Observation, new_id

class PerceptionAgent:
    agent_id="perception_agent"
    version="0.1.0"

    def observe(self, frame: dict) -> Observation:
        obstacle=frame.get("obstacle", {})
        distance=float(obstacle.get("distance_m", 999.0))
        confidence=float(obstacle.get("confidence", 0.0))
        return Observation(
            observation_id=new_id("obs"),
            agent_id=self.agent_id,
            agent_version=self.version,
            timestamp_s=float(frame.get("timestamp_s",0.0)),
            frame_id=str(frame.get("frame_id","base_link")),
            source=str(frame.get("source","simulation")),
            kind="detection",
            confidence=max(0.0,min(1.0,confidence)),
            position_m=[distance, float(obstacle.get("lateral_m",0.0)), 0.0],
            label=str(obstacle.get("label","obstacle")),
            attributes={"distance_m":distance},
            provenance={"producer":self.agent_id,"source_type":"simulation","model_version":"reference-rule-v1"},
        )
