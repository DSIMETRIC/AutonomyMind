from .contracts import WorldState, Observation, new_id

class WorldModelAgent:
    agent_id="world_model_agent"

    def build(self, observation: Observation, ego: dict) -> WorldState:
        attrs=dict(observation.attributes)
        return WorldState(
            world_state_id=new_id("ws"),
            timestamp_s=observation.timestamp_s,
            frame_id=observation.frame_id,
            producer_agent_id=self.agent_id,
            observations=[observation],
            ego=ego,
            map_context={
                "nearest_obstacle_m":attrs.get("distance_m", observation.position_m[0] if observation.position_m else 999.0),
                "object_position_m":list(observation.position_m),
                "object_covariance_diag_m2":attrs.get("covariance_diag_m2"),
                "association_score":attrs.get("association_score"),
                "semantic_label":observation.label,
                "observation_confidence":observation.confidence,
            },
            provenance={
                "producer":self.agent_id,
                "sources":[observation.observation_id],
                "upstream_provenance":observation.provenance,
            },
        )
