from .contracts import AgentProposal, WorldState, new_id

class PlanningAgent:
    agent_id="planning_agent"
    version="0.1.0"

    def propose(self, world: WorldState, robot_id: str="robot-01") -> AgentProposal:
        d=float(world.map_context.get("nearest_obstacle_m",999.0))
        if d < 3.0:
            command={"type":"velocity","vx_mps":0.0,"vy_mps":0.0,"vz_mps":0.0}
            explanation="Obstacle inside planning stop distance; propose HOLD."
            ptype="hold"; confidence=0.98; priority=90
        else:
            command={"type":"velocity","vx_mps":1.0,"vy_mps":0.0,"vz_mps":0.0}
            explanation="Path clear in reference world state; propose forward motion."
            ptype="trajectory"; confidence=0.90; priority=50
        return AgentProposal(
            proposal_id=new_id("proposal"), agent_id=self.agent_id, agent_version=self.version,
            timestamp_s=world.timestamp_s, proposal_type=ptype, target_robot_id=robot_id,
            confidence=confidence, priority=priority, command=command,
            evidence=[f"world_state:{world.world_state_id}"], explanation=explanation
        )
