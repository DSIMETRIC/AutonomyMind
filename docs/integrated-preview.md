# Integrated stationary autonomy preview

One read-only batch producer uses fresh, approximately synchronized stationary Gazebo odometry to create two straight-segment proposals to externally configured goals. The existing DDS reservation service schedules them; the probe verifies membership and geometry, then submits each zero-delay segment to a separate namespaced C++ reference RTA service. A final query checks that the revision remains active. The static scene uses obstacle and peer ground-truth odometry, not collaborative perception. The RTA checks stationary peer assumptions; the reservation check separately models the proposed peer paths. Caller-supplied approximate dynamics are not a validated vehicle model. No command bridge, movement, independent vehicle planning processes, A* or execution gate. Delayed proposals are explicitly unsupported in this preview. A successful result is not a closed-loop demonstration or safety guarantee.

```bash
docker compose -f simulation/gazebo/compose.yaml build gazebo
docker compose -p integrated-preview -f simulation/gazebo/compose.yaml -f simulation/gazebo/compose.fleet-rta.yaml -f simulation/gazebo/compose.integrated-preview.yaml up -d gazebo bridge rta rta-drone reservations
docker compose -p integrated-preview -f simulation/gazebo/compose.yaml -f simulation/gazebo/compose.fleet-rta.yaml -f simulation/gazebo/compose.integrated-preview.yaml run --rm integrated-preview
```

Output: simulation/gazebo/outbox/integrated_preview.json. Domain 82, trusted host network and IPC. Existing RTA base image required; build rta first if absent. No road vehicle RTA profile. Live execution pending.
