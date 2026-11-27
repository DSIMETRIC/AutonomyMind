from .contracts import AgentProposal, new_id
from .conflict import closest_approach, collision_risk

class DynamicPlanningAgent:
    agent_id="dynamic_planning_agent"
    version="0.1.0"

    def generate(self, world: dict, robot_id="robot-01"):
        ego=world["ego"]
        ego_pos=ego.get("position_m",[0.0,0.0,0.0])
        base_speed=float(ego.get("speed_mps",1.0))
        candidates=[
            ("go",{"type":"velocity","vx_mps":base_speed,"vy_mps":0.0,"vz_mps":0.0}),
            ("brake",{"type":"velocity","vx_mps":0.0,"vy_mps":0.0,"vz_mps":0.0}),
            ("left",{"type":"velocity","vx_mps":0.6*base_speed,"vy_mps":0.6,"vz_mps":0.0}),
            ("right",{"type":"velocity","vx_mps":0.6*base_speed,"vy_mps":-0.6,"vz_mps":0.0}),
        ]
        proposals=[]
        for name,cmd in candidates:
            worst_risk=0.0
            evidence=[]
            ego_vel=[cmd["vx_mps"],cmd["vy_mps"],cmd["vz_mps"]]
            for t in world.get("tracks",[]):
                m=closest_approach(ego_pos,ego_vel,t["position_m"],t["velocity_mps"],horizon_s=5.0)
                risk=collision_risk(m,2.0)
                worst_risk=max(worst_risk,risk)
                evidence.append(f"{t['track_id']}:ttc={m['ttc_s']:.2f},d={m['closest_distance_m']:.2f},risk={risk:.3f}")
            priority=int(round(100*(1.0-worst_risk)))
            if name=="brake":
                priority=min(100,priority+10)
            proposals.append(AgentProposal(
                proposal_id=new_id("proposal"),
                agent_id=self.agent_id,
                agent_version=self.version,
                timestamp_s=float(world["timestamp_s"]),
                proposal_type="hold" if name=="brake" else "trajectory",
                target_robot_id=robot_id,
                confidence=max(0.0,min(1.0,1.0-worst_risk)),
                priority=priority,
                command={**cmd,"maneuver":name},
                evidence=evidence,
                explanation=f"Candidate {name} ranked from predicted dynamic conflict risk.",
                model_version="reference-dynamic-planner",
                policy_version="dynamic-planning-v1",
            ))
        return proposals
