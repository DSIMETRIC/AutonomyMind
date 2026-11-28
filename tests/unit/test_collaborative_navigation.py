import unittest
from autonomymind_runtime.collaborative_navigation import propose
from autonomymind_runtime.navigation import Pose2

class NavigationTests(unittest.TestCase):
    def test_shared_proposals_and_expiry(self):
        def record(sender,x,y):
            return {"sender":sender,"source_time":10,"expires":12,"payload":{"label":"lidar_surface_return","frame_id":"world_enu","position_m":[x,y,0]}}
        snap={"frame_id":"world_enu","revision":1,"observations":[record("rover",3,7),record("drone",4,2)]}
        cfg={"map":{"width":12,"height":12,"resolution":1,"origin":[0,0]},"goal":[8.5,2.5],"local_source":"rover","remote_sources":["drone"],"map_prior":"synthetic_free_reference","profile":{"radius_m":.1,"margin_m":.1,"speed_mps":.2,"max_yaw_rate_rad_s":.5,"horizon_s":1,"dt_s":.25,"velocity_samples":3,"yaw_samples":5}}
        r=propose(snap,11,Pose2(2.5,2.5,0),cfg)
        self.assertNotEqual(r["local_route"],r["shared_route"])
        self.assertTrue(r["candidates"])
        self.assertFalse(r["motion_authorized"])
        with self.assertRaises(ValueError):propose(snap,12,Pose2(2.5,2.5,0),cfg)
