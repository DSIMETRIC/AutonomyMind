from .contracts import WorldState, Observation, new_id

class WorldModelAgent:
    agent_id="world_model_agent"

    def build(self, observation: Observation, ego: dict) -> WorldState:
        return WorldState(
            world_state_id=new_id("ws"),
            timestamp_s=observation.timestamp_s,
            frame_id=observation.frame_id,
            producer_agent_id=self.agent_id,
            observations=[observation],
            ego=ego,
            map_context={"nearest_obstacle_m":observation.attributes.get("distance_m",999.0)},
            provenance={"producer":self.agent_id,"sources":[observation.observation_id]},
        )
