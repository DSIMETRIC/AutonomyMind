# autonomymind_decision

`autonomymind_decision` is the deterministic decision and runtime-assurance package for AutonomyMind.

The package keeps operational decision logic in a deterministic C++ core while transport, replay, user interfaces, and optional AI/ML recommendations remain outside the decision-authority boundary.

## Public development scope

This repository contains an open development baseline intended for simulation, software testing, research, and integration work.

The included configuration values are **illustrative development defaults only**. They are not certified, vehicle-approved, mission-approved, or intended to represent requirements from any customer, partner, aircraft, robot, or deployment program.

## Authority boundary

The compiled C++ core owns the decision policy. It evaluates navigation-source availability and integrity, persistence/hysteresis, context events, mission rejoin logic, traffic prediction, threat selection, arbitration, state transitions, and final decision outputs.

Python, dashboards, and higher-level AI components may transport inputs, replay data, display core outputs, or provide advisory recommendations. They do not independently author operational decisions or replace missing operational navigation with validation ground truth.

## Decision priority

The default development policy demonstrates the following sequence:

1. insufficient validated navigation sources -> `HOLD`;
2. navigation integrity below the configured hold criterion -> `HOLD`;
3. traffic conflict -> avoidance candidate evaluation;
4. persistent estimator disagreement -> `VERIFY_NAV`;
5. insufficient redundancy -> `DEGRADE_NAV`;
6. degraded navigation integrity -> `DEGRADE_NAV`;
7. mission cross-track criterion -> `REJOIN_PATH`;
8. otherwise -> `CONTINUE_MISSION`.

Thresholds are configuration-owned under `config/`.

## Libraries and API

- `autonomymind_decision_core`: C++17 API.
- `autonomymind_decision`: shared C ABI.
- `config/baseline.conf`: example development thresholds.
- `config/profiles/autonomymind_core.profile`: default development profile.
- `config/interface.conf`: generated C/C++ interface capacities.

## Build and test

From `autonomy_ws`:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select autonomymind_decision
colcon test --packages-select autonomymind_decision
colcon test-result --verbose
```

See the documents under `docs/` for architecture and validation guidance.

## License

This package is distributed under the Apache License 2.0. See the repository-level `LICENSE` and `NOTICE` files.
