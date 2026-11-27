import json
import unittest
from pathlib import Path
from autonomymind_runtime.industrial_pilot import run
from autonomymind_runtime.mission_orchestrator import MissionOrchestrator,MissionTask
from autonomymind_runtime.fleet import Task,Member

class MissionTests(unittest.TestCase):
    def test_reference_replay(self):
        path=Path(__file__).resolve().parents[2]/"simulation/pilots/industrial_inspection.json"
        s=json.loads(path.read_text())
        r=run(s);self.assertEqual(r,run(s))
        self.assertEqual(r["metrics"]["tasks_completed"],3)
        self.assertEqual(r["metrics"]["tasks_reassigned_to_different_member"],1)
        self.assertEqual(r["metrics"]["rejected_completions"],1)
        self.assertFalse(r["motion_authorized"])
    def test_cyclic_dependency(self):
        with self.assertRaises(ValueError):
            MissionOrchestrator([MissionTask(Task("a",frozenset(),(0,0,0),1),("a",))],2,10)
    def test_expired_epoch_not_accepted(self):
        e=MissionOrchestrator([MissionTask(Task("a",frozenset(),(0,0,0),1))],2,1)
        e.update(Member("m",frozenset(),(0,0,0),(0,0,0),1,0,0),0)
        first=e.reconcile(0)["a"]
        second=e.reconcile(1)["a"]
        self.assertNotEqual(first.epoch,second.epoch)
        self.assertFalse(e.complete("a","m",first.epoch,1))
        self.assertTrue(e.complete("a","m",second.epoch,1))
    def test_variable_fleet(self):
        for count in (1,4,12):
            tasks=[MissionTask(Task(str(i),frozenset(["inspect"]),(i*4,0,0),1)) for i in range(count)]
            e=MissionOrchestrator(tasks,2,10)
            for i in range(count):
                e.update(Member(str(i),frozenset(["inspect"]),(i*4,0,0),(0,0,0),.5,.1,0),0)
            offers=e.reconcile(0)
            self.assertEqual(len(offers),count)
            self.assertEqual(len({x.member_id for x in offers.values()}),count)
