import tempfile, unittest
from pathlib import Path
from autonomymind_runtime.mission_demo import run_mission
from autonomymind_runtime.replay import summarize_trace

SCENARIO="/workspace/simulation/isaac/scenarios/mission_drone_rover_dynamic.json"

class MissionDashboardTraceE2E(unittest.TestCase):
    def test_trace_contains_cooperation_and_dynamic_decision(self):
        with tempfile.TemporaryDirectory() as td:
            trace=Path(td)/"mission_trace.jsonl"
            events=run_mission(SCENARIO,str(trace))
            self.assertEqual(len(events),3)
            summary=summarize_trace(str(trace))
            self.assertEqual(summary["robots"],["drone-01","rover-01"])
            self.assertIn("track-person-01",summary["tracks"])
            self.assertGreaterEqual(summary["assignments"],2)
            self.assertEqual(len(summary["selected_maneuvers"]),3)

if __name__=="__main__": unittest.main()
