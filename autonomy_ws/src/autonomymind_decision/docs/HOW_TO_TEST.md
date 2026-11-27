# How to Test AutonomyMind Decision

## ROS package tests
```bash
cd <autonomymind>/autonomymind_ws
colcon test --packages-select autonomymind_decision --event-handlers console_direct+
colcon test-result --verbose
```

## CI-equivalent standalone tests
```bash
cd <autonomymind>/autonomymind_ws/src/autonomymind_decision
bash tools/test_pipeline.sh
```

## Sanitizers
```bash
SANITIZE=1 \
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
bash tools/test_pipeline.sh
```

## Coverage
- `test_behavior_tree`: candidate generation, RTA gating and deterministic selection.
- `test_mission_behavior_tree`: priority, GPS denied, rejoin and fail safe.
- `test_trajectory_generator`: LEFT/RIGHT physical turn symmetry, yaw/bank evidence, CLIMB.
- `test_full_rta`: traffic, terrain, geofence and flight-envelope rejection.
- `test_threshold_boundaries`: 14.999 / 15.000 / 15.001 m.
- `test_input_validation`: NaN, timestamp regression and disarmed state.
- `test_temporal_scenarios`: nominal -> GPS denied -> recovery -> traffic -> rejoin -> recovery.
- `test_c_api_abi`: ABI version and structure-size contract.

Software tests are not aircraft certification. Flight use requires vehicle- and mission-approved terrain, geofence and envelope constraints plus SIL/HIL/flight evidence.
