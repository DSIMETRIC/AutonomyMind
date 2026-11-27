import json, tempfile, unittest
from pathlib import Path
from autonomymind_runtime.dynamic_pipeline import DynamicAvoidancePipeline

SCENARIO="/workspace/simulation/isaac/scenarios/dynamic_crossing.json"

class DynamicCrossingE2E(unittest.TestCase):
    def test_dynamic_obstacle_avoidance_and_trace(self):
        data=json.loads(Path(SCENARIO).read_text())
        with tempfile.TemporaryDirectory() as td:
            trace=Path(td)/"trace.jsonl"
            pipe=DynamicAvoidancePipeline(trace)
            results=[pipe.step(f) for f in data["frames"]]
            self.assertTrue(trace.exists())
            self.assertEqual(len(trace.read_text().strip().splitlines()),3)
            last=results[-1]
            self.assertTrue(last["selected_safety"]["accepted"])
            self.assertIn(last["selected_proposal"]["command"]["maneuver"],{"brake","left","right"})
            self.assertEqual(len(last["candidates"]),4)

if __name__=="__main__": unittest.main()
