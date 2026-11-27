import unittest
from autonomymind_runtime.cooperative_sim import run,sweep_distance
class CooperativeSimTests(unittest.TestCase):
    def test_crossing_chords(self):
        self.assertAlmostEqual(sweep_distance((-1,0,0),(1,0,0),(0,-1,0),(0,1,0)),0)
    def test_goal_reached_by_motion(self):
        s={"id":"small","dt_s":.5,"steps":12,"margin_m":.1,"goal_tolerance_m":.3,
           "map":{"width":20,"height":20,"resolution":.5,"origin":[-5,-5]},
           "vehicles":[{"id":"a","initial":[-.75,.25,0,0],"profile":{"model":"differential","radius_m":.2,"max_speed_mps":1,"max_turn_rad":1},"capabilities":["inspect"],"uncertainty_m":.05,"sensor_range_m":5}],
           "obstacles":[],"tasks":[{"id":"task","required":["inspect"],"goal":[1.25,.25,0],"priority":1}]}
        r=run(s)
        self.assertEqual(r["metrics"]["tasks_completed"],1)
        self.assertFalse(r["cpp_rta_connected"])
        self.assertFalse(r["motion_authorized"])
