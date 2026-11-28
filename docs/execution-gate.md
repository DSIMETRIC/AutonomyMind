# Gazebo-only rover execution gate

First moving reference: stationary drone and obstacle, straight level rover, goal displacement 0.5 m, speed 0.2 m/s, 0.2 s requested pulses. Each cycle obtains a fresh complete advisory reservation batch, matches it to the request, checks the rover segment with namespaced C++ RTA, checks active revision and lease, and requests a bounded pulse via the existing independent LiDAR relay. No physical hardware adapter. Stop on missing data, rejection, expiry or pose deviation. This is not a hard real-time watchdog: the relay or bridge can fail and leave a latched command; the relay checks receipt freshness and LiDAR but does not independently verify the reservation or RTA. No concurrent clients or untrusted ROS network supported. Approximate dynamics, static peer assumptions and incomplete environmental coverage. No A* or collaborative perception in this moving experiment. No drone motion. No certification.

```bash
docker compose -f simulation/gazebo/compose.yaml build gazebo
docker compose -p execution-gate -f simulation/gazebo/compose.yaml -f simulation/gazebo/compose.closed_loop.yaml -f simulation/gazebo/compose.fleet-rta.yaml -f simulation/gazebo/compose.execution-gate.yaml up -d gazebo bridge rta reservations watchdog
docker compose -p execution-gate -f simulation/gazebo/compose.yaml -f simulation/gazebo/compose.closed_loop.yaml -f simulation/gazebo/compose.fleet-rta.yaml -f simulation/gazebo/compose.execution-gate.yaml run --rm execution-gate
```

Output simulation/gazebo/outbox/execution_gate.json. Domain 83 on trusted isolated simulation host only. Stop the stack after the experiment. Live run pending.
