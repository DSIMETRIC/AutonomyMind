import tempfile
import unittest
from pathlib import Path
from autonomymind_runtime.local_reasoning import LocalModel,ReasoningRanker,interpret_scene
from autonomymind_runtime.model_backend import ModelRegistry,ModelSpec
class Fake:
    model="test"
    def __init__(self,reply):self.reply=reply
    def ask(self,*args,**kw):return self.reply
class ReasoningTests(unittest.TestCase):
    def test_remote_endpoint_rejected(self):
        for url in ("https://example.com","http://example.com","http://localhost:11434/path"):
            with self.assertRaises(ValueError):LocalModel(url,"test")
    def test_ranking(self):
        registry=ModelRegistry()
        registry.register(ModelSpec("test","1","local-llm","0"*64),ReasoningRanker(Fake({"ranked_ids":["b","a"],"explanation":"test"})))
        self.assertFalse(registry.advise("test","1",{"candidate_ids":["a","b"]})["motion_authorized"])
    def test_invalid_rank(self):
        registry=ModelRegistry()
        registry.register(ModelSpec("test","1","local-llm","0"*64),ReasoningRanker(Fake({"ranked_ids":["c"],"explanation":"test"})))
        with self.assertRaises(ValueError):registry.advise("test","1",{"candidate_ids":["a"]})
    def test_scene_is_not_geometry(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/"frame.jpg";p.write_bytes(b"\xff\xd8\xfffixture")
            result=interpret_scene(Fake({"objects":["vehicle"],"hazards":[],"summary":"test"}),p)
            self.assertFalse(result["metric_geometry_available"])
            self.assertFalse(result["motion_authorized"])
            with self.assertRaises(ValueError):interpret_scene(Fake({"objects":[],"hazards":[],"summary":"","command":"go"}),p)
