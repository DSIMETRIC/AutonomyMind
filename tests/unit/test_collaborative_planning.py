import unittest
from autonomymind_runtime.collaborative_planning import compare_routes

class CollaborativePlanningTests(unittest.TestCase):
    def test_remote_obstacle_changes_route(self):
        def record(sender,x,y,expires=12):
            return {"sender":sender,"source_time":10,"expires":expires,"payload":{"label":"lidar_surface_return","frame_id":"world_enu","position_m":[x,y,0]}}
        snapshot={"frame_id":"world_enu","revision":1,"observations":[record("rover",3,7),record("drone",4,2)]}
        config={"map":{"width":12,"height":12,"resolution":1,"origin":[0,0]},"start":[2.5,2.5],"goal":[8.5,2.5],"clearance_m":.2,"local_source":"rover","remote_sources":["drone"],"map_prior":"synthetic_free_reference"}
        r=compare_routes(snapshot,11,config)
        self.assertEqual(r["status"],"alternate_route")
        self.assertTrue(r["invalidated_local_cells"])
        self.assertFalse(r["motion_authorized"])
        self.assertFalse(r["cpp_rta_assessed"])
        snapshot["observations"][1]["expires"]=11
        r=compare_routes(snapshot,11,config)
        self.assertFalse(r["route_changed"])
        self.assertEqual(r["remote_additional_cells"],0)
