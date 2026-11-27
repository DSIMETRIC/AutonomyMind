import unittest
from autonomymind_runtime.conflict import closest_approach, collision_risk

class ConflictTests(unittest.TestCase):
    def test_crossing_conflict(self):
        m=closest_approach([0,0,0],[1,0,0],[4,2,0],[0,-1,0],5.0)
        self.assertLess(m["closest_distance_m"],2.0)
        self.assertGreater(collision_risk(m,2.0),0.0)

    def test_non_conflict(self):
        m=closest_approach([0,0,0],[1,0,0],[4,5,0],[0,1,0],5.0)
        self.assertEqual(collision_risk(m,2.0),0.0)

if __name__=="__main__": unittest.main()
