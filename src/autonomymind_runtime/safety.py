from .contracts import AgentProposal, SafetyCheck, SafetyDecision, new_id

class DeterministicSafetyBridge:
    validator_id="deterministic_safety_bridge"

    def validate(self, proposal: AgentProposal, obstacle_distance_m: float=None) -> SafetyDecision:
        vx=float(proposal.command.get("vx_mps",0.0))
        vy=float(proposal.command.get("vy_mps",0.0))
        vz=float(proposal.command.get("vz_mps",0.0))
        values=[vx,vy,vz]
        if obstacle_distance_m is not None:
            values.append(float(obstacle_distance_m))
        finite=all(x==x and abs(x)!=float("inf") for x in values)
        speed=(vx*vx+vy*vy+vz*vz)**0.5
        speed_ok=speed <= 2.0
        clearance_ok=True if obstacle_distance_m is None else ((float(obstacle_distance_m)>=2.0) or speed<=1e-9)
        checks=[
            SafetyCheck("finite_input",finite,1.0 if finite else 0.0,1.0),
            SafetyCheck("example_speed_limit",speed_ok,speed,2.0),
            SafetyCheck("example_obstacle_clearance",clearance_ok,999.0 if obstacle_distance_m is None else float(obstacle_distance_m),2.0),
        ]
        accepted=all(c.passed for c in checks)
        return SafetyDecision(
            decision_id=new_id("safety"), proposal_id=proposal.proposal_id,
            timestamp_s=proposal.timestamp_s, validator_id=self.validator_id,
            accepted=accepted, status="accepted" if accepted else "rejected",
            reason="all deterministic checks passed" if accepted else "one or more deterministic checks failed",
            checks=checks,
        )

    def validate_candidates(self, proposals: list[AgentProposal], obstacle_distance_m: float=None):
        return [(p,self.validate(p,obstacle_distance_m)) for p in proposals]
