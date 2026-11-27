import unittest
import math
from autonomymind_runtime.sensor_evidence import project_scan,route_from_hits
from autonomymind_runtime.navigation import OccupancyGrid
class SensorEvidenceTests(unittest.TestCase):
    def test_projection(self):
        hits=project_scan([2.,float("inf"),.01],0.,.1,.08,12.,(1.,2.,0.,math.pi/2))
        self.assertEqual(len(hits),1)
        self.assertAlmostEqual(hits[0][1][0],1.)
        self.assertAlmostEqual(hits[0][1][1],4.45)
    def test_changed_route(self):
        g=OccupancyGrid(40,40,.5,(-10.25,-10.25))
        g.free={(x,y) for x in range(g.width) for y in range(g.height)}
        before=route_from_hits(g,(0.,0.),(6.,0.),[],.65)
        after=route_from_hits(g,(0.,0.),(6.,0.),[[3.75,0.,.65]],.65)
        self.assertTrue(before and after)
        self.assertNotEqual(before,after)
