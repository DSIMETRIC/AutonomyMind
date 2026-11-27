# Decision Authority Architecture

## Scope

AutonomyMind separates deterministic decision authority from transport, visualization, replay, and advisory AI/ML components.

This is an open reference architecture for software development and testing. It is not a certification artifact and does not encode requirements for a specific customer, partner, vehicle, or deployment program.

## Deterministic authority

The compiled C++ core evaluates navigation evidence, context, mission state, traffic, candidate trajectories, and configured runtime-assurance checks.

Supporting components may:

- marshal inputs;
- replay synthetic or recorded test data;
- serialize outputs;
- visualize evidence;
- provide advisory rankings.

Supporting components must not silently replace unavailable operational inputs with validation ground truth.

## Example decision flow

The development policy demonstrates fail-closed behavior, navigation assurance, traffic avoidance, mission rejoin, and deterministic arbitration.

All thresholds and limits are external configuration values and are provided only as example software-test defaults. Real-world use requires independent engineering review and deployment-specific values.

## Validation boundary

Expected labels and synthetic truth data may be used to evaluate output during testing, but they must not participate in runtime decision generation.
