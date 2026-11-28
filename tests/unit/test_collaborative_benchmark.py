import unittest
from autonomymind_runtime.collaborative_benchmark import benchmark

def record(sender,x,y,expires=12):
    return {"sender":sender,"source_time":10,"expires":expires,"payload":{"label":"lidar_surface_return","frame_id":"world_enu","position_m":[x,y,0]}}

class BenchmarkTests(unittest.TestCase):
    def config(self):
        return {"map":{"width":12,"height":12,"resolution":1,"origin":[0,0]},"start":[2.5,2.5],"goal_axes":[[2.5,4.5,8.5],[2.5]],"clearance_m":.2,"local_source":"rover","remote_sources":["drone"],"map_prior":"synthetic_free_reference"}
    def test_outcomes_and_expiry(self):
        snap={"frame_id":"world_enu","revision":1,"observations":[record("rover",3,7),record("drone",4,2)]}
        r=benchmark(snap,11,self.config())
        self.assertEqual(r["counts"],{"unchanged":1,"no_shared_route":1,"alternate_route":1})
        self.assertTrue(r["cases"][2]["causes"])
        self.assertEqual(r["cases"][2]["causes"][0]["sources"],["drone"])
        self.assertFalse(r["motion_authorized"])
        self.assertFalse(r["cpp_rta_assessed"])
        snap["observations"][1]["expires"]=11
        r=benchmark(snap,11,self.config())
        self.assertEqual(r["counts"],{"unchanged":3})
    def test_other_map_and_sources(self):
        c=self.config()
        c.update(map={"width":16,"height":10,"resolution":.5,"origin":[-2,-2]},start=[-.25,-.25],goal_axes=[[2.75],[-.25]],clearance_m=.1,local_source="a",remote_sources=["b"])
        snap={"frame_id":"world_enu","revision":2,"observations":[record("a",0,2),record("b",1,-.25)]}
        r=benchmark(snap,11,c)
        self.assertEqual(r["counts"],{"alternate_route":1})
        self.assertEqual(r["cases"][0]["causes"][0]["sources"],["b"])
