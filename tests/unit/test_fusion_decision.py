import unittest
from autonomymind_runtime.contracts import Observation, AgentProposal
from autonomymind_runtime.sensor_fusion import SensorFusionAgent
from autonomymind_runtime.decision import DecisionAgent

class FusionDecisionTests(unittest.TestCase):
    def test_fusion_preserves_provenance(self):
        a=Observation("a","p1","1",1.0,"base","camera","detection",0.8,[4,0,0],[0,0,0],"box",{"distance_m":4},{"producer":"p1"})
        b=Observation("b","p2","1",1.0,"base","lidar","detection",1.0,[3,0,0],[0,0,0],"box",{"distance_m":3},{"producer":"p2"})
        fused=SensorFusionAgent().fuse([a,b])
        self.assertEqual(fused.position_m[0],3)
        self.assertAlmostEqual(fused.confidence,0.9)
        self.assertEqual(fused.provenance["sources"],["a","b"])

    def test_decision_selects_priority_and_links_upstream(self):
        low=AgentProposal("p1","planner","1",0,"trajectory","r",0.95,20,{"vx_mps":1},[],"low")
        high=AgentProposal("p2","coord","1",0,"hold","r",0.70,90,{"vx_mps":0},[],"high")
        selected=DecisionAgent().select([low,high])
        self.assertEqual(selected.proposal_type,"hold")
        self.assertIn("upstream_proposal:p2",selected.evidence)
        self.assertEqual(selected.agent_id,"decision_agent")

if __name__=="__main__": unittest.main()
