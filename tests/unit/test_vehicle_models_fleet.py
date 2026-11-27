import math
import unittest
from autonomymind_runtime.vehicle_models import State, Control, Profile, ModelRegistry
from autonomymind_runtime.fleet import Fleet, Member, Task

class VehicleFleetTests(unittest.TestCase):
    def test_ackermann_cannot_turn_in_place(self):
        r=ModelRegistry();s=State(0,0,0,0);p=Profile("ackermann",2,4,.5,wheelbase_m=2.5)
        self.assertEqual(r.propagate(s,Control(turn=.3),p,1),s)
        a=r.propagate(s,Control(2,turn=.3),p,1)
        self.assertAlmostEqual(a.yaw,2*math.tan(.3)/2.5)
        self.assertGreater(a.y,0)
    def test_rover_and_drone(self):
        r=ModelRegistry();s=State(0,0,0,0)
        self.assertAlmostEqual(r.propagate(s,Control(turn=.5),Profile("differential",.3,1,1),1).yaw,.5)
        self.assertEqual(r.propagate(s,Control(vertical=1),Profile("multirotor",.4,2,1,2),1).z,1)
        with self.assertRaises(ValueError):r.propagate(s,Control(),Profile("fixed_wing",1,10,1,2,min_speed_mps=3),1)
    def test_invalid_controls(self):
        with self.assertRaises(ValueError):ModelRegistry().propagate(State(0,0,0,0),Control(100),Profile("ackermann",2,5,.5,wheelbase_m=2),1)
    def test_fleet_reassignment(self):
        f=Fleet(2)
        def member(id,cap,t):return Member(id,frozenset(cap),(0,0,0),(0,0,0),1,.1,t)
        f.update(member("a",["road"],0));f.update(member("b",["road"],1));f.update(member("c",["air"],1))
        tasks=[Task("delivery",frozenset(["road"]),(1,0,0),10),Task("survey",frozenset(["air"]),(1,0,3),5)]
        self.assertEqual(f.allocate(tasks,1)[0],{"delivery":"a","survey":"c"})
        self.assertEqual(f.allocate(tasks,2.5)[0],{"delivery":"b","survey":"c"})
        self.assertEqual(f.conflicts(2.5,2,.1)["unknown_peers"],["a"])
    def test_variable_fleet_capacity(self):
        for count in (1,3,12):
            f=Fleet(2)
            for i in range(count):f.update(Member(str(i),frozenset(["survey"]),(i*4,0,0),(0,0,0),.5,.1,1))
            tasks=[Task(str(i),frozenset(["survey"]),(i,0,0),1) for i in range(count+1)]
            a,u=f.allocate(tasks,1)
            self.assertEqual(len(set(a.values())),count);self.assertEqual(len(u),1)
