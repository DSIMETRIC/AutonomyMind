import unittest
from autonomymind_runtime.perception import PerceptionAgent
from autonomymind_runtime.world_model import WorldModelAgent
from autonomymind_runtime.planning import PlanningAgent

class AgentTests(unittest.TestCase):
    def test_perception_to_world_state(self):
        frame={"timestamp_s":1.0,"frame_id":"base_link","source":"test","ego":{"robot_id":"r1"},
               "obstacle":{"label":"box","distance_m":4.0,"confidence":0.9}}
        obs=PerceptionAgent().observe(frame)
        self.assertEqual(obs.kind,"detection")
        self.assertEqual(obs.position_m[0],4.0)
        ws=WorldModelAgent().build(obs,frame["ego"])
        self.assertEqual(ws.map_context["nearest_obstacle_m"],4.0)

    def test_planner_holds_near_obstacle(self):
        frame={"timestamp_s":1.0,"frame_id":"base_link","source":"test","ego":{"robot_id":"r1"},
               "obstacle":{"distance_m":1.0,"confidence":0.9}}
        obs=PerceptionAgent().observe(frame)
        ws=WorldModelAgent().build(obs,frame["ego"])
        proposal=PlanningAgent().propose(ws,"r1")
        self.assertEqual(proposal.proposal_type,"hold")
        self.assertEqual(proposal.command["vx_mps"],0.0)

if __name__=="__main__": unittest.main()
