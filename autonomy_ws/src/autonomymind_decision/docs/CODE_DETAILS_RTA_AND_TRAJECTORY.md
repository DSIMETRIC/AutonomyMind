# Code Details: Full RTA and Physical Candidate Trajectories

## Main files
- `trajectory_generator.cpp`: deterministic candidate trajectories.
- `rta.cpp`: independent Full RTA gates.
- `behavior_tree.cpp`: candidate orchestration and safe-candidate selection.
- `mission_behavior_tree.cpp`: top-level deterministic arbitration.
- `core.cpp`: evidence construction and runtime integration.
- `c_api.cpp`: C ABI evidence export.

## Turn profile
For normalized time `u=t/T`, the generator uses `q(u)=10u^3-15u^4+6u^5`.

For LEFT/RIGHT:
```text
heading(t) = heading_0 +/- turn_angle * q(u)
yaw_rate(t) = +/- turn_angle/T * q'(u)
vx = Vh * cos(heading)
vy = Vh * sin(heading)
```
Position is trapezoidally integrated. The coordinated-turn bank estimate is `atan2(Vh*abs(yaw_rate), g)`.

## RTA
Threat velocity is reconstructed as `v_threat = v_relative + v_ownship`. Every sampled candidate point is checked for traffic separation, terrain clearance, geofence containment and aircraft-envelope limits.

Safety is a hard constraint. Only `rta_safe == true` candidates can be selected. Among fully safe candidates the deterministic objective is maximum predicted minimum traffic separation.

## ABI
The C ABI exposes version/structure-size guards and per-candidate RTA evidence. HMI consumers display evidence; they do not recompute safety policy.
