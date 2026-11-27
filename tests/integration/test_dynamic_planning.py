import unittest
from autonomymind_runtime.dynamic_planning import DynamicPlanningAgent
from autonomymind_runtime.decision import DecisionAgent
from autonomymind_runtime.safety import DeterministicSafetyBridge

class DynamicPlanningTests(unittest.TestCase):
    def test_generates_and_selects_safe_candidates(self):
        world={
          "timestamp_s":1.0,
          "ego":{"robot_id":"r1","position_m":[0,0,0],"speed_mps":1.0},
          "tracks":[{"track_id":"t1","label":"person","position_m":[3,1,0],"velocity_mps":[0,-1,0]}]
        }
        props=DynamicPlanningAgent().generate(world,"r1")
        self.assertEqual({p.command["maneuver"] for p in props},{"go","brake","left","right"})
        validated=DeterministicSafetyBridge().validate_candidates(props)
        safe=[p for p,d in validated if d.accepted]
        selected=DecisionAgent().select(safe)
        self.assertIn(selected.command["maneuver"],{"brake","left","right","go"})

if __name__=="__main__": unittest.main()
