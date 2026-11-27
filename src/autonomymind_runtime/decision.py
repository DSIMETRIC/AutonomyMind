from .contracts import AgentProposal, new_id

class DecisionAgent:
    agent_id="decision_agent"
    version="0.2.0"

    def select(self, proposals: list[AgentProposal]) -> AgentProposal:
        if not proposals:
            raise ValueError("at least one proposal is required")
        # Prefer higher priority/confidence, then lower-speed fallback for equal score.
        def key(p):
            speed=abs(float(p.command.get("vx_mps",0.0)))+abs(float(p.command.get("vy_mps",0.0)))+abs(float(p.command.get("vz_mps",0.0)))
            return (p.priority,p.confidence,-speed)
        selected=max(proposals,key=key)
        return AgentProposal(
            proposal_id=new_id("decision"),
            agent_id=self.agent_id,
            agent_version=self.version,
            timestamp_s=selected.timestamp_s,
            proposal_type=selected.proposal_type,
            target_robot_id=selected.target_robot_id,
            confidence=selected.confidence,
            priority=selected.priority,
            command=dict(selected.command),
            evidence=[*selected.evidence,f"upstream_proposal:{selected.proposal_id}"],
            explanation=f"Selected {selected.command.get('maneuver','candidate')} proposal from {selected.agent_id}.",
            model_version="reference-deterministic-selector",
            policy_version="decision-agent-v2",
        )
