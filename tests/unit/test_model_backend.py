import unittest
from autonomymind_runtime.model_backend import ModelRegistry,ModelSpec,ReferenceRanker
class ModelBackendTests(unittest.TestCase):
    def registry(self,backend):
        r=ModelRegistry();r.register(ModelSpec("test","1","reference","0"*64),backend);return r
    def test_reference(self):
        r=self.registry(ReferenceRanker())
        q={"candidate_ids":["a","b"],"scores":{"a":1,"b":2}}
        result=r.advise("test","1",q)
        self.assertEqual(result["ranked_ids"],["b","a"])
        self.assertTrue(result["requires_safety_validation"])
        self.assertFalse(result["motion_authorized"])
        self.assertEqual(result,r.advise("test","1",q))
    def test_injected_candidate(self):
        class Bad:
            def infer(self,q):return {"ranked_ids":["invented"],"explanation":""}
        with self.assertRaises(ValueError):self.registry(Bad()).advise("test","1",{"candidate_ids":["a"]})
    def test_no_authority_override(self):
        class Bad:
            def infer(self,q):return {"ranked_ids":["a"],"explanation":"","motion_authorized":True}
        with self.assertRaises(ValueError):self.registry(Bad()).advise("test","1",{"candidate_ids":["a"]})
