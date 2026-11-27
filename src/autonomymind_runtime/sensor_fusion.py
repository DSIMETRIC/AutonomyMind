from .contracts import Observation, new_id

class SensorFusionAgent:
    agent_id="sensor_fusion_agent"
    version="0.1.0"

    def fuse(self, observations: list[Observation]) -> Observation:
        if not observations:
            raise ValueError("at least one observation is required")
        primary=min(observations,key=lambda o: float(o.attributes.get("distance_m",999.0)))
        confidence=sum(o.confidence for o in observations)/len(observations)
        return Observation(
            observation_id=new_id("fused"),
            agent_id=self.agent_id,
            agent_version=self.version,
            timestamp_s=max(o.timestamp_s for o in observations),
            frame_id=primary.frame_id,
            source="fusion",
            kind=primary.kind,
            confidence=max(0.0,min(1.0,confidence)),
            position_m=list(primary.position_m),
            velocity_mps=list(primary.velocity_mps),
            label=primary.label,
            attributes={**primary.attributes,"source_count":len(observations)},
            provenance={
                "producer":self.agent_id,
                "source_type":"fusion",
                "sources":[o.observation_id for o in observations],
            },
        )
