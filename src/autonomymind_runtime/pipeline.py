from .perception import PerceptionAgent
from .sensor_fusion import SensorFusionAgent
from .world_model import WorldModelAgent
from .planning import PlanningAgent
from .decision import DecisionAgent
from .safety import DeterministicSafetyBridge
from .semanticmap import SemanticMapConnector

class AutonomyPipeline:
    def __init__(self, outbox: str):
        self.perception=PerceptionAgent()
        self.fusion=SensorFusionAgent()
        self.world=WorldModelAgent()
        self.planning=PlanningAgent()
        self.decision=DecisionAgent()
        self.safety=DeterministicSafetyBridge()
        self.semantic=SemanticMapConnector(outbox)

    def step(self, frame: dict) -> dict:
        raw_obs=self.perception.observe(frame)
        fused_obs=self.fusion.fuse([raw_obs])
        world=self.world.build(fused_obs,frame.get("ego",{"robot_id":"robot-01"}))
        planning_proposal=self.planning.propose(world,str(world.ego.get("robot_id","robot-01")))
        decision_proposal=self.decision.select([planning_proposal])
        safety=self.safety.validate(decision_proposal,float(world.map_context["nearest_obstacle_m"]))
        annotation=self.semantic.maybe_export(frame,raw_obs.to_dict())
        return {
            "observation":raw_obs.to_dict(),
            "fused_observation":fused_obs.to_dict(),
            "world_state":world.to_dict(),
            "planning_proposal":planning_proposal.to_dict(),
            "proposal":decision_proposal.to_dict(),
            "safety_decision":safety.to_dict(),
            "annotation_request":annotation,
        }
