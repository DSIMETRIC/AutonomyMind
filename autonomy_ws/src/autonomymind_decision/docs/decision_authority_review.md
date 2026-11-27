# AutonomyMind R9.16 Decision Authority Review

## Non-negotiable architecture

The **compiled C++ Unified Core is the sole operational decision authority**.

Production Python is passive. It may read/normalize recorded inputs, marshal raw inputs into the C ABI, replay telemetry, serialize Core outputs, and expose factual source availability. It must not select `nav_mode`, author `decision`, `reason`, `guidance`, `nav_health` or `active_rule`, apply policy thresholds, implement persistence/hysteresis/arbitration, or replace unavailable NAV inputs with validation ground truth.

The dashboard is presentation-only and renders the Core/API outputs rather than reconstructing operational state.

## C++ decision chain reviewed

`core/src/core.cpp` computes source validity, NAV mode, integrity, context events, mission geometry, traffic prediction, arbitration and final decision. Priority is:

1. insufficient validated NAV sources -> `HOLD / NAV_UNAVAILABLE`;
2. NAV integrity below HOLD criterion -> `HOLD / NAV_INTEGRITY_HOLD`;
3. traffic conflict -> avoidance;
4. persistent estimator disagreement -> `VERIFY_NAV`;
5. insufficient redundancy -> `DEGRADE_NAV`;
6. degraded NAV integrity -> `DEGRADE_NAV`;
7. mission cross-track criterion -> `REJOIN_PATH`;
8. otherwise -> `CONTINUE_MISSION`.

The controlled baseline declares one source as the minimum operational count and two as the required redundancy count. These states are produced by the C++ Core, not by Python.

## Ground truth

Validation or ground-truth data must not silently replace unavailable operational NAV inputs. Replay tooling may compare Core results against expected outcomes, but expected labels must not participate in runtime decision generation.
