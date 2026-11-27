from .contracts import AgentProposal, SafetyCheck, SafetyDecision, new_id

class DeterministicSafetyBridge:
    validator_id="deterministic_safety_bridge"

    def validate(self, proposal: AgentProposal, obstacle_distance_m: float) -> SafetyDecision:
        vx=float(proposal.command.get("vx_mps",0.0))
        finite = all(map(lambda x: x == x and abs(x) != float("inf"), [vx, obstacle_distance_m]))
        speed_ok=abs(vx) <= 2.0
        clearance_ok=(obstacle_distance_m >= 2.0) or vx <= 0.0
        checks=[
            SafetyCheck("finite_input",finite,1.0 if finite else 0.0,1.0),
            SafetyCheck("example_speed_limit",speed_ok,abs(vx),2.0),
            SafetyCheck("example_obstacle_clearance",clearance_ok,obstacle_distance_m,2.0),
        ]
        accepted=all(c.passed for c in checks)
        return SafetyDecision(
            decision_id=new_id("safety"), proposal_id=proposal.proposal_id,
            timestamp_s=proposal.timestamp_s, validator_id=self.validator_id,
            accepted=accepted, status="accepted" if accepted else "rejected",
            reason="all deterministic checks passed" if accepted else "one or more deterministic checks failed",
            checks=checks,
        )
