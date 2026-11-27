import unittest
from autonomymind_runtime.dynamic_pipeline import DynamicAvoidancePipeline

def frame(tracks):
    return {"timestamp_s":1.0,"ego":{"robot_id":"r","position_m":[0,0,0],"speed_mps":1.0},"tracks":tracks}

class DynamicAuthorization(unittest.TestCase):
    def test_crossing_rejects_go(self):
        r=DynamicAvoidancePipeline().step(frame([{"track_id":"p","position_m":[4,0,0],"velocity_mps":[0,0,0]}]))
        go=next(c for c in r["candidates"] if c["proposal"]["command"]["maneuver"]=="go")
        self.assertFalse(go["safety"]["accepted"])
        self.assertIsNotNone(r["selected_proposal"])
        self.assertEqual(r["selected_proposal"]["proposal_id"],r["selected_safety"]["proposal_id"])

    def test_no_safe_candidate(self):
        r=DynamicAvoidancePipeline().step(frame([{"track_id":"p","position_m":[0,0,0],"velocity_mps":[0,0,0]}]))
        self.assertIsNone(r["selected_proposal"])
        self.assertEqual(r["authorization"],"denied_no_candidate")

    def test_invalid_frame(self):
        f=frame([])
        f["data_valid"]=False
        self.assertIsNone(DynamicAvoidancePipeline().step(f)["selected_proposal"])

    def test_missing_tracks(self):
        f=frame([])
        del f["tracks"]
        self.assertIsNone(DynamicAvoidancePipeline().step(f)["selected_proposal"])
