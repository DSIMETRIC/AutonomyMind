# Gazebo Harmonic reference integration

This isolated Docker Compose stack uses Gazebo Harmonic through the ROS Jazzy ros-gz packages. It is an initial read-only integration, not closed-loop autonomy. The static primitive models publish simulator ground-truth odometry; the probe forwards a rover snapshot and derives relative positions of the other two models for the existing C++ RTA service. A stationary candidate is expected to pass and a candidate ending at the obstacle to be rejected. No movement command is sent.

From the repository root:

```bash
docker compose -f simulation/gazebo/compose.yaml build rta
docker compose -f simulation/gazebo/compose.yaml build gazebo
docker compose -f simulation/gazebo/compose.yaml up -d rta gazebo bridge
docker compose -f simulation/gazebo/compose.yaml run --rm probe
```

Build order is intentional: the Gazebo image extends the locally built ROS core image. Inspect logs with docker compose -f simulation/gazebo/compose.yaml logs --tail=80. Stop with docker compose -f simulation/gazebo/compose.yaml down. Assessment output: simulation/gazebo/outbox/assessment.json. Restart the stack before repeating the probe because the RTA tracks accepted sequence numbers.

ROS domain 74 and an ordinary Compose network separate this experiment from the existing host-network RTA. This is not a security isolation guarantee. World coordinates are designated ENU for this synthetic scene. No GPU is required for server-only execution. Camera/LiDAR/IMU, moving models, independent source-time and health validation, dynamic trajectories, and actual command feedback are future integration tasks. No connection to physical hardware. The test uses simplified static ground truth and illustrative candidate extrema, not measured vehicle dynamics. This stack needs validation on a Docker host.
