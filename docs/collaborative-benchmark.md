# Collaborative planning benchmark

The previous live single-goal experiment (goal 6,-3) produced identical routes, 9 local cells, 13 union cells and 4 remote-only cells, with no invalidated route cells (reported world revision 1156). This is a valid negative result for route benefit, not a perception failure.

The benchmark enumerates every Cartesian goal from a configurable pair of axes (default 42 goals, including the original goal) on one frozen fresh DDS snapshot. It reports unchanged, alternate_route, no_shared_route, local_unreachable and unexpected changed_without_invalidation separately; it records remote cells and source identities whose inflated footprint intersects invalidated route cells. It does not search for a favorable snapshot or exclude blocked goals. PASS means enumeration completed with fresh independent evidence, not that any alternative was demonstrated. A blocked goal is included in no_shared_route; this does not prove a corridor blockage. The synthetic regressions test expiry, unchanged, blocked goal, alternate route and another map/source configuration.

```bash
docker compose run --build --rm test
docker compose -f simulation/gazebo/compose.yaml build gazebo
docker compose -p collaborative-perception -f simulation/gazebo/compose.collaborative-perception.yaml up -d --force-recreate gazebo bridge world rover-sensor drone-sensor
docker compose -p collaborative-perception -f simulation/gazebo/compose.collaborative-perception.yaml run --rm benchmark
```

Report: simulation/gazebo/outbox/collaborative_benchmark.json. Routes are reference planar A* proposals under a declared synthetic free prior; no validated free-space inference, dynamic motion, RTA authorization or production safety claim. Evaluation time is frozen for all goals, so results are retrospective rather than live commands. Tests and live run pending.
