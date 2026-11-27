import json, unittest
from pathlib import Path
from autonomymind_runtime.cooperative_pipeline import CooperativePipeline

SCENARIO="/workspace/simulation/isaac/scenarios/cooperative_drone_rover.json"

class CooperativeE2E(unittest.TestCase):
    def test_drone_rover_assignment_and_shared_world(self):
        data=json.loads(Path(SCENARIO).read_text())
        result=CooperativePipeline().step(data["frames"][0])
        self.assertEqual(result["assignments"]["task-survey"],"drone-01")
        self.assertEqual(result["assignments"]["task-inspect"],"rover-01")
        self.assertEqual(len(result["shared_world"]["robots"]),2)
        self.assertEqual(len(result["shared_world"]["tracks"]),1)

if __name__=="__main__": unittest.main()
