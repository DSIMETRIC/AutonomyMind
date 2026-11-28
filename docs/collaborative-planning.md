# Collaborative route planning — read-only reference

The new planner consumes a single fresh shared world snapshot and generates two A* proposals from the same declared free-map prior, start, goal and inflated clearance: local-only occupied evidence and the union of selected local and remote sources. PASS requires remote evidence to invalidate at least one cell on the local route and a nonempty alternate shared route. An unchanged route is not a failure of perception; it simply does not demonstrate a planning benefit for the configured goal.

The scenario JSON defines a reference start and goal (not waypoints). The probe reads live DDS observations; no obstacle positions are supplied to the planner. The configured start is the reference stationary rover's nominal pose, not independently measured live pose. It may be changed along with the goal to test other missions. Unknown space is not certified free: the experiment deliberately uses a synthetic map prior.

Rebuild the Gazebo image and recreate the existing collaborative-perception stack, then run:

```bash
docker compose -f simulation/gazebo/compose.yaml build gazebo
docker compose -p collaborative-perception -f simulation/gazebo/compose.collaborative-perception.yaml up -d --force-recreate gazebo bridge world rover-sensor drone-sensor
docker compose -p collaborative-perception -f simulation/gazebo/compose.collaborative-perception.yaml run --rm planning-probe
```

Output: simulation/gazebo/outbox/collaborative_planning.json. The live scene may not yield the required route change; inspect the report rather than changing evidence to force PASS. Unit and live tests pending. No time reservations, vehicle motion, dynamic prediction, probabilistic fusion, RTA assessment or actuator authority in this milestone. The planner is deterministic; it is not learned inference.
