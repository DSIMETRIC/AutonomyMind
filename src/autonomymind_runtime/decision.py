from .contracts import AgentProposal, new_id

class DecisionAgent:
    agent_id="decision_agent"
    version="0.1.0"

    def select(self, proposals: list[AgentProposal]) -> AgentProposal:
        if not proposals:
            raise ValueError("at least one proposal is required")
        selected=max(proposals,key=lambda p:(p.priority,p.confidence))
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
            explanation=f"Selected proposal from {selected.agent_id}: {selected.explanation}",
            model_version="reference-deterministic-selector",
            policy_version="decision-agent-v1",
        )
