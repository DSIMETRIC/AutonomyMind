# Collaborative navigation preview

Read-only integration of live stationary rover/drone LiDAR producers, shared DDS evidence, A*, sampled unicycle local candidates, advisory reservations and rover C++ reference assessment. Uses observed odometry and a configurable goal, not predefined waypoints. Both global and local checks use the same inflated clearance. The selected candidate is nonstationary, but never executed. A turn-only proposal is possible. The drone is a stationary reservation peer, not an assessed moving participant. Remote evidence need not change this particular route; the preceding benchmark separately demonstrated that benefit.

Use the existing collaborative-perception stack with the additional overlay:

```bash
docker compose -f simulation/gazebo/compose.yaml build gazebo
docker compose -p collaborative-perception -f simulation/gazebo/compose.collaborative-perception.yaml -f simulation/gazebo/compose.collaborative-navigation.yaml up -d --force-recreate gazebo bridge world rover-sensor drone-sensor navigation-rta navigation-reservations
docker compose -p collaborative-perception -f simulation/gazebo/compose.collaborative-perception.yaml -f simulation/gazebo/compose.collaborative-navigation.yaml run --rm navigation-probe
```

Output: simulation/gazebo/outbox/collaborative_navigation.json. Reference map prior is synthetic free space, occupied cells are planar returns, RTA uses cell centers as stationary threats and approximate caller-supplied dynamics. No coverage guarantee, no moving perception, no actuator bridge, no full dynamic obstacle prediction. A rejection is not a successful acceptance. No physical deployment. Tests and live run pending.
