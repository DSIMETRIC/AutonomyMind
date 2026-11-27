import unittest
from autonomymind_runtime.shared_world import SharedWorld
from autonomymind_runtime.dds_protocol import envelope
class SharedWorldTests(unittest.TestCase):
    def observation(self,sender,stamp=10,frame="world_enu"):
        return envelope(sender,"*","world.observation",{"observation_id":"one","timestamp_s":stamp,"frame_id":frame,"position_m":[1,2,0],"velocity_mps":[0,0,0],"confidence":.8,"uncertainty_m":.2,"label":"obstacle"},stamp,10)
    def test_multi_source_and_expiry(self):
        w=SharedWorld()
        for sender in ("drone","rover","car"):self.assertEqual(w.ingest(self.observation(sender),10),"accepted")
        self.assertEqual(len(w.snapshot(10)["observations"]),3)
        self.assertEqual(w.ingest(self.observation("drone"),10),"duplicate_or_old")
        self.assertEqual(w.snapshot(15)["observations"],[])
        self.assertFalse(w.snapshot(15)["coverage_complete"])
    def test_frame_and_stale(self):
        w=SharedWorld()
        self.assertEqual(w.ingest(self.observation("a",frame="camera"),10),"frame_mismatch")
        self.assertEqual(w.ingest(self.observation("a"),16),"stale")
    def test_capacity(self):
        w=SharedWorld(capacity=1)
        self.assertEqual(w.ingest(self.observation("a"),10),"accepted")
        self.assertEqual(w.ingest(self.observation("b"),10),"capacity")
    def test_semantics_not_geometry(self):
        w=SharedWorld()
        m=envelope("vision","*","scene.semantic",{"kind":"unverified_semantic_description","metric_geometry_available":False,"motion_authorized":False,"requires_independent_validation":True,"image_sha256":"a"*64,"description":{"objects":["vehicle"],"hazards":[],"summary":"test"}},10)
        self.assertEqual(w.ingest(m,10),"accepted")
        s=w.snapshot(10)
        self.assertEqual(s["observations"],[])
        self.assertEqual(len(s["semantic_reports"]),1)
        self.assertFalse(s["motion_authorized"])
