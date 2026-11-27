# Collaborative perception — stationary Gazebo reference

Two independently running sensor producers consume separate simulated LiDAR and odometry streams and send bounded, timestamped surface returns to a shared DDS evidence service. The probe constructs a planar occupied-cell union, preserves source identity, and requires at least one additional cell from the drone beyond rover evidence. No object association, probabilistic fusion or free-space inference is claimed. Both vehicles remain stationary; the drone is a static low-altitude sensor platform, not a flight dynamics model. The scene and sensor extrinsics are reference configuration. No ROS-to-Gazebo command bridge is present.

```bash
docker compose -f simulation/gazebo/compose.yaml build rta
docker compose -f simulation/gazebo/compose.yaml build gazebo
docker compose -p collaborative-perception -f simulation/gazebo/compose.collaborative-perception.yaml up -d gazebo bridge world rover-sensor drone-sensor
docker compose -p collaborative-perception -f simulation/gazebo/compose.collaborative-perception.yaml run --rm probe
```

Output: simulation/gazebo/outbox/collaborative_perception.json. Domain 81, host networking and IPC for trusted local development only. Stop conflicting stacks. Producer-specific timestamps are simulator stamps plus wall ingestion time. Reports expire after two seconds; no authenticated identities or synchronized multi-sensor capture. Returns are capped to 48 nearest occupied 0.25 m cells per capture and producer. Planar projection assumes level stationary vehicles. The cell union is not a traversability map, does not account for height or covariance, and is not connected to the planner or RTA in this experiment. No road vehicle sensor source, camera inference, movement or physical safety validation. Build and live test pending.
