# AutonomyMind Decision Architecture: Behavior Trees, Physical Candidate Trajectories and Full RTA

## Purpose
The safety-relevant runtime is deterministic C++. Python and dashboards are presentation/transport layers and do not own thresholds, arbitration or safety policy.

## Runtime flow
```text
Perception / Navigation / Mission
            |
            v
   Situation Evaluation
            |
            v
   MissionDecisionTree
            |
            v
 CollisionAvoidanceTree
            |
 LEFT / RIGHT / CLIMB candidates
            |
            v
 Physical Trajectory Generator
            |
            v
       FullRtaChecker
   /        |       |        \
Traffic  Terrain  Geofence  Flight envelope
   \        |       |        /
             PASS
              |
              v
 Select validated candidate
              |
              v
      Decision / Guidance
```

## Physical LEFT / RIGHT model
LEFT and RIGHT are no longer lateral translations. Horizontal speed is preserved while heading transitions smoothly through a configured turn angle using a quintic profile. Yaw rate is derived from the heading profile; lateral acceleration is `v * yaw_rate`; bank angle is estimated with `atan(a_lateral/g)`. Candidate position is integrated deterministically at the configured prediction step.

This is a coordinated-turn kinematic approximation, not a 6-DoF aerodynamic aircraft model.

## Full RTA gates
A candidate is selectable only when all gates pass:
1. predicted 3-D traffic separation;
2. terrain clearance;
3. 3-D geofence containment;
4. flight envelope: speed, vertical speed, acceleration, jerk, yaw rate and bank angle.

The baseline terrain model is a flat elevation plane and the geofence is an axis-aligned 3-D volume. Mission-specific terrain/geofence providers can replace those data sources without changing RTA authority semantics.

## Fail safe
Invalid input produces HOLD. Disarmed state produces STANDBY/NO_COMMAND. A traffic conflict with no fully RTA-valid candidate produces HOLD. GPS denied without a validated alternate navigation source produces HOLD.
