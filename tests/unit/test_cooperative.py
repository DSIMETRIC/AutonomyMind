import unittest
from autonomymind_runtime.cooperative import CoordinationAgent, RobotState, MissionTask

class CooperativeTests(unittest.TestCase):
    def test_capability_based_assignment(self):
        robots=[
            RobotState("drone-01","drone",[0,0,0],[0,0,0],["survey","inspect"],True),
            RobotState("rover-01","rover",[5,0,0],[0,0,0],["inspect","carry"],True),
        ]
        tasks=[
            MissionTask("survey","survey",[10,0,0],["survey"],90),
            MissionTask("inspect","inspect",[6,0,0],["inspect"],70),
        ]
        a=CoordinationAgent().assign(tasks,robots)
        self.assertEqual(a["survey"],"drone-01")
        self.assertEqual(a["inspect"],"rover-01")

    def test_inter_robot_conflict_detection(self):
        robots=[
            RobotState("a","drone",[0,0,0],[1,0,0],[],True),
            RobotState("b","rover",[4,0,0],[-1,0,0],[],True),
        ]
        c=CoordinationAgent().inter_robot_conflicts(robots,horizon_s=5.0,min_sep_m=1.0)
        self.assertEqual(len(c),1)
        self.assertLess(c[0]["separation_m"],1.0)

if __name__=="__main__": unittest.main()
