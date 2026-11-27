import json, tempfile, unittest
from pathlib import Path
from autonomymind_runtime.isaac_exchange import run_exchange_jsonl
from autonomymind_runtime.replay import summarize_trace

class IsaacExchangeTests(unittest.TestCase):
    def test_jsonl_exchange_becomes_dashboard_trace(self):
        frame={
          "timestamp_s":0.0,
          "robots":[
            {"robot_id":"drone-01","robot_type":"drone","position_m":[0,0,4],"velocity_mps":[0,0,0],"capabilities":["survey"],"available":True},
            {"robot_id":"rover-01","robot_type":"rover","position_m":[0,0,0],"velocity_mps":[1,0,0],"capabilities":["inspect"],"available":True}
          ],
          "tasks":[],
          "tracks":[{"track_id":"t1","label":"person","position_m":[3,1,0],"velocity_mps":[0,-1,0],"confidence":0.9}]
        }
        with tempfile.TemporaryDirectory() as td:
            inp=Path(td)/"frames.jsonl"; out=Path(td)/"trace.jsonl"
            inp.write_text(json.dumps(frame)+"\n",encoding="utf-8")
            self.assertEqual(run_exchange_jsonl(str(inp),str(out)),1)
            s=summarize_trace(str(out))
            self.assertEqual(s["robots"],["drone-01","rover-01"])
            self.assertEqual(s["tracks"],["t1"])

if __name__=="__main__": unittest.main()
