import unittest
from autonomymind_runtime.spatiotemporal import Request,Coordinator,conflict

def req(name,priority,a,b):
    return Request(name,priority,.3,.1,((0,*a),(4,*b)))

class ReservationTests(unittest.TestCase):
    def test_crossing_priority_and_expiry(self):
        a=req("a",2,(-2,0,0),(2,0,0));b=req("b",1,(0,-2,0),(0,2,0))
        c=Coordinator(["a","b"],12,.5,6,.2)
        self.assertIsNotNone(conflict(a,0,b,0,12,.2))
        r=c.plan([a,b],0,0,5)
        self.assertEqual(r["status"],"scheduled")
        self.assertGreater(r["reservations"][1]["delay_s"],0)
        self.assertFalse(r["motion_authorized"])
        with self.assertRaises(ValueError):c.plan([a,b],1,0,5)
        c.advance(5)
        self.assertIsNone(c.current)
        self.assertEqual(c.revision,2)
    def test_swap_and_altitude(self):
        a=req("a",2,(-2,0,0),(2,0,0));b=req("b",1,(2,0,0),(-2,0,0))
        c=Coordinator(["a","b"],12,.5,6,.2)
        self.assertEqual(c.plan([a,b],0,0,5)["status"],"unscheduled")
        d=req("d",1,(0,-2,3),(0,2,3))
        self.assertIsNone(conflict(a,0,d,0,12,.2))
    def test_missing_peer(self):
        c=Coordinator(["a","b"],12,.5,6,.2)
        with self.assertRaises(ValueError):c.plan([req("a",2,(-2,0,0),(2,0,0))],0,0,5)

    def test_lease_exceeds_horizon(self):
        c=Coordinator(["a"],5,.5,1,.2)
        a=req("a",1,(0,0,0),(1,0,0))
        with self.assertRaisesRegex(ValueError,"invalid lease"):
            c.plan([a],0,0,8)
        self.assertEqual(c.plan([a],0,0,3)["status"],"scheduled")
