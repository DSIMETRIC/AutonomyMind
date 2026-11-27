import unittest
from autonomymind_runtime.collaborative_evidence import occupied_sources
from autonomymind_runtime.navigation import OccupancyGrid
class CollaborativeEvidenceTests(unittest.TestCase):
    def test_union_and_expiry(self):
        def rec(sender,x,expires=12):
            return {"sender":sender,"source_time":10,"expires":expires,"payload":{"label":"lidar_surface_return","frame_id":"world_enu","position_m":[x,0,0]}}
        snapshot={"frame_id":"world_enu","observations":[rec("rover",1),rec("drone",1),rec("drone",2),rec("old",3,10)]}
        cells=occupied_sources(snapshot,OccupancyGrid(10,10,1,(0,0)),11)
        self.assertEqual(cells,{(1,0):{"rover","drone"},(2,0):{"drone"}})
