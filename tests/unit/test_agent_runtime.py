import json
import unittest
from pathlib import Path
from autonomymind_runtime.agent_runtime import Runtime,AgentSpec,Envelope
from autonomymind_runtime.agent_pilot import replay

class AgentRuntimeTests(unittest.TestCase):
    def setup_runtime(self):
        r=Runtime();received=[]
        r.register(AgentSpec("a","source","1",(),(),("data",)),lambda e,emit:None,10)
        r.register(AgentSpec("b","worker","1",("work",),("data",),()),lambda e,emit:received.append(e.payload),10)
        return r,received
    def test_delivery_and_duplicates(self):
        r,received=self.setup_runtime()
        r.send("a","b","data",{"x":1},2)
        r.enqueue(r.queue[0]);r.drain()
        self.assertEqual(received,[{"x":1}])
        self.assertEqual(r.trace[-1]["status"],"duplicate")
    def test_expiry_and_contract(self):
        r,received=self.setup_runtime()
        r.send("a","b","data",{},1);r.advance(1);r.drain()
        self.assertEqual(r.trace[-1]["status"],"invalid_or_expired")
        r.send("a","b","bad",{},2);r.drain()
        self.assertEqual(r.trace[-1]["status"],"contract_mismatch")
        self.assertFalse(received)
        r.advance(10);self.assertEqual(r.discover("work"),[])
        r.heartbeat("b",2);self.assertEqual(r.discover("work"),["b"])
    def test_pilot(self):
        scenario=json.loads((Path(__file__).resolve().parents[2]/"simulation/pilots/industrial_inspection.json").read_text())
        result=replay(scenario)
        self.assertEqual(result["report"]["metrics"]["tasks_completed"],3)
        self.assertEqual(len(result["agent_trace"]),3)
        self.assertTrue(all(x["status"]=="delivered" for x in result["agent_trace"]))
        self.assertFalse(result["report"]["motion_authorized"])
