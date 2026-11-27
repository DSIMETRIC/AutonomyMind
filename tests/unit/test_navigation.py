import math
import random
import unittest
from autonomymind_runtime.navigation import *

class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.profile=NavigationProfile(.20,.10,.5,1.,2.,.1,3,7)
    def grid(self):
        g=OccupancyGrid(40,40,.25,(-5,-5))
        g.free={(x,y) for x in range(40) for y in range(40)}
        return g
    def test_unknown_not_traversable(self):
        g=OccupancyGrid(40,40,.25,(-5,-5))
        self.assertEqual(global_path(g,(0,0),(2,2),.3),[])
    def test_ray(self):
        g=OccupancyGrid(40,40,.25,(-5,-5))
        g.observe_ray((0,0),0,2,4)
        self.assertIn(g.cell(2,0),g.occupied)
        self.assertIn(g.cell(1,0),g.free)
    def test_random_obstacle_routes(self):
        rng=random.Random(42)
        for _ in range(12):
            g=self.grid()
            x=rng.randint(15,24);y=rng.randint(15,24)
            g.occupied={(i,j) for i in range(x-2,x+3) for j in range(y-2,y+3)}
            route=global_path(g,(-3,0),(3,0),.3)
            self.assertTrue(route)
            self.assertTrue(all(g.traversable(g.cell(*p),.3) for p in route))
    def test_dynamic_obstacle_rejection(self):
        g=self.grid();p=Pose2(0,0,0)
        clear=local_candidates(g,p,(2,0),self.profile)
        blocked=local_candidates(g,p,(2,0),self.profile,[DynamicObstacle(0,0,0,0,.2,.1)])
        self.assertTrue(clear);self.assertFalse(blocked)
    def test_replanning(self):
        g=self.grid();p=Pose2(-3,0,0)
        route,options=replan(g,p,(3,0),self.profile)
        self.assertTrue(route);self.assertTrue(options)
        g.occupied.add(g.cell(*route[len(route)//2]))
        new,_=replan(g,p,(3,0),self.profile)
        self.assertTrue(new);self.assertNotEqual(route,new)
