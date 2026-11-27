import unittest
from autonomymind_runtime.contracts import AgentProposal
from autonomymind_runtime.safety import DeterministicSafetyBridge

def proposal(vx):
    return AgentProposal("p1","planner","0.1",0.0,"trajectory","r1",0.9,50,
                         {"type":"velocity","vx_mps":vx},["ws:1"],"test")

class SafetyTests(unittest.TestCase):
    def test_rejects_forward_motion_too_close(self):
        d=DeterministicSafetyBridge().validate(proposal(1.0),1.0)
        self.assertFalse(d.accepted)
        self.assertEqual(d.status,"rejected")

    def test_accepts_hold_near_obstacle(self):
        d=DeterministicSafetyBridge().validate(proposal(0.0),1.0)
        self.assertTrue(d.accepted)
        self.assertTrue(d.deterministic)

    def test_rejects_example_speed_limit(self):
        d=DeterministicSafetyBridge().validate(proposal(3.0),10.0)
        self.assertFalse(d.accepted)

if __name__=="__main__": unittest.main()
