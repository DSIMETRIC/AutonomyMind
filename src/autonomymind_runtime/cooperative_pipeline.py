from .cooperative import SharedWorldModel, CoordinationAgent, RobotState, MissionTask

class CooperativePipeline:
    def __init__(self):
        self.world=SharedWorldModel()
        self.coord=CoordinationAgent()

    def step(self, frame: dict):
        robots=[RobotState(**r) for r in frame.get("robots",[])]
        tasks=[MissionTask(**t) for t in frame.get("tasks",[])]
        for r in robots:
            self.world.update_robot(r)
        for t in frame.get("tracks",[]):
            self.world.update_track(t)
        assignments=self.coord.assign(tasks,robots)
        conflicts=self.coord.inter_robot_conflicts(robots)
        directives=[]
        for c in conflicts:
            # Transparent deterministic coordination policy:
            # higher lexical robot_id yields, preserving one robot as leader.
            yield_robot=max(c["a"],c["b"])
            directives.append({
                "robot_id":yield_robot,
                "directive":"yield",
                "reason":f"predicted inter-robot separation {c['separation_m']:.2f} m at {c['ttc_s']:.2f} s",
            })
        return {
            "shared_world":self.world.snapshot(),
            "assignments":assignments,
            "conflicts":conflicts,
            "coordination_directives":directives,
        }
