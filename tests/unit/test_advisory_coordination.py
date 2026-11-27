import unittest
from autonomymind_runtime.mission_orchestrator import MissionOrchestrator,MissionTask
from autonomymind_runtime.fleet import Task,Member
class AdvisoryTests(unittest.TestCase):
    def make(self,ranker):
        engine=MissionOrchestrator([MissionTask(Task("inspect",frozenset(["survey"]),(0,0,0),1))],2,10,ranker)
        for id,caps,x,t in [("a",["survey"],1,3),("b",["survey"],2,3),("c",["road"],0,3),("d",["survey"],0,0)]:
            engine.update(Member(id,frozenset(caps),(x,0,0),(0,0,0),1,0,t),3)
        return engine
    def test_rank_only_eligible(self):
        def rank(q):
            self.assertEqual(q["candidate_ids"],["a","b"])
            return {"ranked_ids":["b","a"],"motion_authorized":False,"requires_safety_validation":True}
        e=self.make(rank)
        self.assertEqual(e.reconcile(3)["inspect"].member_id,"b")
    def test_reject_invented_or_stale(self):
        e=self.make(lambda q:{"ranked_ids":["d","a"],"motion_authorized":False,"requires_safety_validation":True})
        self.assertEqual(e.reconcile(3),{})
        self.assertEqual(e.events[-1]["kind"],"model_advice_rejected")
    def test_failure_no_offer(self):
        def fail(q):raise RuntimeError("model failed")
        self.assertEqual(self.make(fail).reconcile(3),{})
