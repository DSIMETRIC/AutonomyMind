"""Isaac Sim standalone reference scene exporter.

Kinematic scripted motion, ground-truth poses only. No actuation, detector,
LiDAR or IMU integration. Run in an Isaac Sim Python environment.
"""
import argparse
import json
from pathlib import Path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--steps", type=int, default=60)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    if not 2 <= args.steps <= 2400:
        parser.error("steps must be between 2 and 2400")
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": args.headless})
    try:
        import omni.usd
        from pxr import Gf, UsdGeom
        context = omni.usd.get_context()
        if not context.open_stage(str(Path(args.scene).resolve())):
            raise RuntimeError("could not open scene")
        app.update()
        stage = context.get_stage()
        if stage is None:
            raise RuntimeError("missing USD stage")
        prims = {name: stage.GetPrimAtPath("/World/" + name)
                 for name in ("Rover", "Drone", "MovingObstacle")}
        if not all(p.IsValid() for p in prims.values()):
            raise RuntimeError("scene prim missing")
        ops = {name: UsdGeom.Xformable(p).GetOrderedXformOps()[0]
               for name, p in prims.items()}
        out = Path(args.output)
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("w", encoding="utf-8") as stream:
            for i in range(args.steps):
                t = i / 24.0
                poses = {"Rover": (-4 + 0.5*t, 0, 0.35),
                         "Drone": (-4 + 0.5*t, 0, 3),
                         "MovingObstacle": (0, 3 - 0.5*t, 0.5)}
                for name, xyz in poses.items():
                    ops[name].Set(Gf.Vec3d(*xyz))
                app.update()
                frame = {"schema_version": "1.0", "source": "isaac_usd_scripted_ground_truth",
                         "timestamp_s": t, "frame_id": "world_enu",
                         "robots": [
                             {"robot_id": "rover-01", "robot_type": "rover", "position_m": list(poses["Rover"]),
                              "velocity_mps": [0.5,0,0], "capabilities": ["inspect"], "available": True},
                             {"robot_id": "drone-01", "robot_type": "drone", "position_m": list(poses["Drone"]),
                              "velocity_mps": [0.5,0,0], "capabilities": ["survey"], "available": True}],
                         "tracks": [{"track_id": "obstacle-01", "label": "obstacle",
                                     "position_m": list(poses["MovingObstacle"]), "velocity_mps": [0,-0.5,0],
                                     "confidence": 1.0}], "tasks": []}
                stream.write(json.dumps(frame, allow_nan=False) + "\n")
        print("Wrote scripted scene frames:", out)
    finally:
        app.close()

if __name__ == "__main__":
    main()
