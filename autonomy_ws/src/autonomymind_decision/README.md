# autonomymind_decision

`autonomymind_decision` is the deterministic decision-authority package for AutonomyMind. It integrates the validated R9.16/v18 C++ Unified Core into the main ROS 2 workspace while keeping transport, replay, UI and AI recommendations outside the operational decision boundary.

## Authority boundary

The compiled C++ core is the sole owner of operational decision policy. It computes NAV source validity and mode, integrity, persistence/hysteresis, context events, mission rejoin logic, traffic prediction, threat selection, arbitration, state transitions and final decision outputs.

Python, dashboards and higher-level AI components may transport inputs, replay data, display Core outputs or provide recommendations. They must not independently author operational `decision`, `reason`, `guidance`, `nav_health` or `active_rule`, apply policy thresholds, or replace missing operational navigation with validation ground truth.

## Decision priority

1. insufficient validated NAV sources -> `HOLD`;
2. NAV integrity below the hold criterion -> `HOLD`;
3. traffic conflict -> `AVOID_LEFT` / `AVOID_RIGHT`;
4. persistent estimator disagreement -> `VERIFY_NAV`;
5. insufficient redundancy -> `DEGRADE_NAV`;
6. degraded NAV integrity -> `DEGRADE_NAV`;
7. mission cross-track criterion -> `REJOIN_PATH`;
8. otherwise -> `CONTINUE_MISSION`.

Thresholds are configuration-owned under `config/`.

## Libraries and API

- `autonomymind_decision_core`: C++17 API.
- `autonomymind_decision`: shared C ABI.
- `config/baseline.conf`: controlled operational thresholds.
- `config/profiles/autonomymind_core.profile`: default profile.
- `config/interface.conf`: generated C/C++ interface capacities.

The Core returns the decision and evidence including reason, guidance, NAV mode/health, active rule, event flags, integrity/residual values, mission rejoin data, traffic separation data, transition progress, explanations, evaluation rows and configuration SHA-256.

## Build and test

From `autonomymind_ws`:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select autonomymind_decision
colcon test --packages-select autonomymind_decision
colcon test-result --verbose
```

See [`docs/decision_authority_review.md`](docs/decision_authority_review.md) and [`../../../docs/architecture/decision_design.md`](../../../docs/architecture/decision_design.md).
