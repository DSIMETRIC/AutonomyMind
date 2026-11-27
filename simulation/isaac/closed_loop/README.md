# Closed-loop simulation boundary

This milestone provides reference point-clearance authorization, not a certified runtime assurance implementation. No actuator interface is enabled. The existing JSONL exchange is offline, not a live ROS bridge.

Next integration acceptance criteria:
- Explicit simulator version and compatible GPU.
- USD scene with physical robot models, calibrated camera/LiDAR and ROS 2 clock/odometry.
- Time-aligned tracks in a common frame with freshness and source health.
- Independent C++ RTA integration and vehicle-specific dynamics, braking distance, uncertainty and communication watchdog.
- Simulator-only command adapter with explicit opt-in and no physical robot routing.
- SIL scenarios covering collision, sensor dropout, stale state, no accepted candidate and recovery.
- Real dashboard trace and recorded simulator capture.

The Python reference gate assumes point geometry and instantaneous constant velocity, with an illustrative five-second horizon and two-metre clearance. Its acceptance is not evidence that a real maneuver is safe.

## ROS 2 C++ assessment endpoint

Run from the repository root:

```bash
docker compose -f simulation/isaac/closed_loop/compose.rta.yaml up --build
```

The node consumes nav_msgs/Odometry on /autonomymind_sim/simulation/odometry and serves /autonomymind_sim/simulation/assess_trajectory using AssessSimulationTrajectory. The service calls VehicleRta through SimulationAdapter. It binds a request to the odometry stamp and checks local receipt age, identity, frame and sequence. Its result is a reference assessment only; no command is emitted. Synthetic profiles are hardcoded for reference testing, not operational use.

Configure the Isaac ROS bridge to publish odometry in world_enu (with child-frame twist) and use the same ROS domain (73 by default). A separate simulated perception/planning producer must provide sampled relative trajectories and relative threat vectors in world ENU. No Isaac USD scene, OmniGraph wiring or producer is included yet. The DDS smoke test uses synthetic odometry and requests, not Isaac. Host networking requires an appropriately isolated simulation network. Receipt age is not source freshness; repeated old odometry can appear fresh. Request health flags and peak dynamics are caller assertions, not independent evidence. The node does not control vehicles or issue fallback commands.

## Reference USD scene and offline export

Open simulation/isaac/worlds/cooperative_reference.usda in a compatible Isaac Sim installation. The scene uses original primitive geometry, a flat collision floor, kinematic rover/drone/obstacle bodies and camera prims. It is not a flight dynamics or wheel-controller model.

Using the Isaac Sim installation's Python launcher, run (paths relative to repository root):

```bash
/path/to/isaac-sim/python.sh simulation/isaac/closed_loop/export_scene.py \
  --scene simulation/isaac/worlds/cooperative_reference.usda \
  --output simulation/isaac/exchange/scene_frames.jsonl --steps 60 --headless
```

The exporter updates USD transforms and produces scripted ground-truth frames; it does not advance a closed-loop physics controller or read rendered pixels. To convert the exchange to the existing dashboard trace:

```bash
docker compose -f compose.demo.yaml build
docker compose -f compose.demo.yaml run --rm \
  -v "$PWD/simulation/isaac/exchange:/exchange:ro" mission \
  isaac-exchange --input /exchange/scene_frames.jsonl --trace /trace/mission_trace.jsonl
docker compose -f compose.demo.yaml up --no-deps dashboard
```

This offline trace still uses the Python reference gate, not the C++ ROS service. The new scene and exporter have not been run on an Isaac GPU host. LiDAR, IMU, rendered RGB inference, ROS graph, actual dynamics, independent source health and simulator feedback remain open acceptance items. The ROS assessment service is a separate endpoint, not connected by this exporter. No physical commands are issued.
