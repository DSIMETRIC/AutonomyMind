# Live Gazebo sensor to shared world reference experiment

This read-only integration uses actual simulated GPU LiDAR returns and odometry, projects finite hits with the fixed reference SDF sensor extrinsic, sends versioned observations to an independent DDS world service, retrieves a snapshot, and compares A* routes before and after the sensor evidence. The first segment is submitted to the existing C++ rover assessment service. No velocity command is published. PASS requires a changed nonempty route and a completed C++ assessment, not RTA acceptance or physical movement.

```bash
docker compose -f simulation/gazebo/compose.yaml build rta
docker compose -f simulation/gazebo/compose.yaml build gazebo
docker compose -p sensor-world -f simulation/gazebo/compose.yaml -f simulation/gazebo/compose.closed_loop.yaml -f simulation/gazebo/compose.sensor-world.yaml up -d rta gazebo bridge world
docker compose -p sensor-world -f simulation/gazebo/compose.yaml -f simulation/gazebo/compose.closed_loop.yaml -f simulation/gazebo/compose.sensor-world.yaml run --rm sensor-world-probe
```

Output: simulation/gazebo/outbox/sensor_world.json. Domain 80, host network and IPC, trusted Linux only. Stop conflicting stacks first. The route goal and map bounds are external JSON; no fixed waypoints. The free-space map is a declared synthetic prior, not a sensor-derived occupancy map. Returns represent surface points, not object centers or tracks; velocity is unknown and zero is a stationary scene assumption only. Gazebo time is retained separately from wall-clock ingestion time. Single stationary rover, no drone sensing, cross-vehicle fusion, autonomous motion, camera interpretation or independent coverage verification. The C++ service receives static projected returns and estimated dynamic peaks; this is not a safety authorization and does not establish feasibility of the A* route for differential-drive motion. Build, unit tests and live probe not yet executed.
