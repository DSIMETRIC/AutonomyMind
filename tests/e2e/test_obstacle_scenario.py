import tempfile, unittest
from pathlib import Path
from autonomymind_runtime.isaac_adapter import IsaacScenarioAdapter
from autonomymind_runtime.pipeline import AutonomyPipeline

SCENARIO="/workspace/simulation/isaac/scenarios/obstacle_demo.json"

class ObstacleScenarioE2E(unittest.TestCase):
    def test_complete_vertical(self):
        frames=IsaacScenarioAdapter().load(SCENARIO)
        with tempfile.TemporaryDirectory() as td:
            results=[AutonomyPipeline(td).step(f) for f in frames]
            self.assertEqual(len(results),3)
            self.assertEqual(results[0]["proposal"]["proposal_type"],"trajectory")
            self.assertEqual(results[-1]["proposal"]["proposal_type"],"hold")
            self.assertTrue(results[-1]["safety_decision"]["accepted"])
            self.assertTrue(list(Path(td).glob("annotation-*.json")))

if __name__=="__main__": unittest.main()
