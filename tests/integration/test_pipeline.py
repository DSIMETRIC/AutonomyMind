import tempfile, unittest
from pathlib import Path
from autonomymind_runtime.pipeline import AutonomyPipeline

class PipelineIntegrationTests(unittest.TestCase):
    def test_low_confidence_exports_annotation_request(self):
        with tempfile.TemporaryDirectory() as td:
            frame={"timestamp_s":2.0,"frame_id":"base_link","source":"simulation",
                   "ego":{"robot_id":"rover-01"},
                   "obstacle":{"label":"box","distance_m":1.5,"confidence":0.55}}
            result=AutonomyPipeline(td).step(frame)
            self.assertIsNotNone(result["annotation_request"])
            self.assertTrue(Path(result["annotation_request"]).exists())
            self.assertEqual(result["proposal"]["proposal_type"],"hold")
            self.assertTrue(result["safety_decision"]["accepted"])

    def test_high_confidence_does_not_export(self):
        with tempfile.TemporaryDirectory() as td:
            frame={"timestamp_s":0.0,"frame_id":"base_link","source":"simulation",
                   "ego":{"robot_id":"rover-01"},
                   "obstacle":{"distance_m":8.0,"confidence":0.95}}
            result=AutonomyPipeline(td).step(frame)
            self.assertIsNone(result["annotation_request"])
            self.assertEqual(result["proposal"]["proposal_type"],"trajectory")

if __name__=="__main__": unittest.main()
