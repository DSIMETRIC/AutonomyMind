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

## Mobile sensor milestone

A separate world defines a two-wheel differential drive rover, RGB camera, GPU LiDAR, IMU and odometry, with stationary drone and obstacle reference geometry. The existing verified static scenario remains unchanged. The mobile bridge is read-only: the Gazebo DiffDrive command topic is not bridged to ROS. No autonomous movement or RTA-based actuation is enabled.

Run with a separate Compose project:

```bash
docker compose -p autonomymind-mobile -f simulation/gazebo/compose.yaml -f simulation/gazebo/compose.mobile.yaml build rta
docker compose -p autonomymind-mobile -f simulation/gazebo/compose.yaml -f simulation/gazebo/compose.mobile.yaml build gazebo
docker compose -p autonomymind-mobile -f simulation/gazebo/compose.yaml -f simulation/gazebo/compose.mobile.yaml up -d rta gazebo bridge
docker compose -p autonomymind-mobile -f simulation/gazebo/compose.yaml -f simulation/gazebo/compose.mobile.yaml run --rm probe
```

The sensor probe requires three messages from each stream and basic image/range/IMU validity; results are saved to outbox/sensors.json. Headless Ogre2 uses software rendering for the first attempt, which may be unsupported or slow on some hosts; inspect simulator logs if RGB/LiDAR are absent. Do not treat sensor messages as validated object tracks. Physical wheel behavior, drone flight, moving obstacles, perception fusion and closed-loop RTA execution remain untested. No physical actuator connections.

## Experimental closed-loop rover
Separate Compose project and ROS domain 76. The simulator-only bridge enables one command topic. The world raises the LiDAR above the chassis and adds a front caster to improve stability. The probe checks advancing source stamps, receipt ages, alignment, pose, and forward scan; it calls the C++ reference RTA before each short forward pulse, and publishes zero between pulses and on exit. The reference threats are stationary simulator ground truth, not detected tracks. This is a bounded straight-line experiment, not a navigation stack. No independent watchdog exists if the process is killed or DDS fails; stop the simulation on failure. No physical hardware or host networking is configured. Do not connect this domain or topic to hardware.

Run the base build first if needed:
```bash
docker compose -f simulation/gazebo/compose.yaml build rta
docker compose -f simulation/gazebo/compose.yaml build gazebo
docker compose -p autonomymind-loop -f simulation/gazebo/compose.yaml -f simulation/gazebo/compose.closed_loop.yaml up -d rta gazebo bridge
docker compose -p autonomymind-loop -f simulation/gazebo/compose.yaml -f simulation/gazebo/compose.closed_loop.yaml run --rm probe
```
Stop/reset: same Compose flags with down. Results: outbox/closed_loop.json. Restart before repeat. PASS requires measured forward displacement, obstacle-related stop, final clearance and low longitudinal velocity. No collision-contact monitor, continuous-time assurance, independent watchdog or calibrated braking model yet. Neither this world nor probe has been executed on a simulator host.
