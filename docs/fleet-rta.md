# Gazebo fleet RTA reference assessment

The existing C++ ROS assessment node now exposes its reference profile and numerical limits as ROS parameters. Existing defaults are preserved. Two namespaced instances can independently assess a stationary rover and multirotor from Gazebo odometry, using distinct reference profiles. The probe forwards simulator truth and checks stationary candidates plus intentionally invalid speed/descent proposals. It does not publish velocity commands.

Run after stopping conflicting domain-76 experiments:

```bash
docker compose -f simulation/gazebo/compose.yaml build rta
docker compose -f simulation/gazebo/compose.yaml build gazebo
docker compose -p fleet-rta -f simulation/gazebo/compose.yaml -f simulation/gazebo/compose.closed_loop.yaml -f simulation/gazebo/compose.fleet-rta.yaml up -d rta rta-drone gazebo bridge
docker compose -p fleet-rta -f simulation/gazebo/compose.yaml -f simulation/gazebo/compose.closed_loop.yaml -f simulation/gazebo/compose.fleet-rta.yaml run --rm fleet-probe
```

Output: simulation/gazebo/outbox/fleet_rta.json. The reference scene contains a static drone, not a flying vehicle. The bridge odometry is simulator ground truth; the probe assumes stationary threats and does not establish synchronized sensor coverage. The service still accepts caller-reported dynamic metrics and is not an actuator authorization. Profiles are illustrative parameters, not certified limits. Ackermann-specific steering and braking checks, live multisensor world model, DDS planning agents, and actual cooperative movement remain pending. Build and execution have not been verified.
