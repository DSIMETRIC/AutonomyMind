from .perception import PerceptionAgent
from .world_model import WorldModelAgent
from .planning import PlanningAgent
from .safety import DeterministicSafetyBridge
from .semanticmap import SemanticMapConnector

class AutonomyPipeline:
    def __init__(self, outbox: str):
        self.perception=PerceptionAgent()
        self.world=WorldModelAgent()
        self.planning=PlanningAgent()
        self.safety=DeterministicSafetyBridge()
        self.semantic=SemanticMapConnector(outbox)

    def step(self, frame: dict) -> dict:
        obs=self.perception.observe(frame)
        world=self.world.build(obs,frame.get("ego",{"robot_id":"robot-01"}))
        proposal=self.planning.propose(world,str(world.ego.get("robot_id","robot-01")))
        decision=self.safety.validate(proposal,float(world.map_context["nearest_obstacle_m"]))
        annotation=self.semantic.maybe_export(frame,obs.to_dict())
        return {"observation":obs.to_dict(),"world_state":world.to_dict(),
                "proposal":proposal.to_dict(),"safety_decision":decision.to_dict(),
                "annotation_request":annotation}
