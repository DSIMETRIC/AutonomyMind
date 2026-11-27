# How to Test AutonomyMind Decision

## ROS package tests

```bash
cd <autonomymind>/autonomy_ws
colcon test --packages-select autonomymind_decision --event-handlers console_direct+
colcon test-result --verbose
```

## CI-equivalent standalone tests

```bash
cd <autonomymind>/autonomy_ws/src/autonomymind_decision
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

The suite exercises behavior-tree selection, trajectory generation, runtime-assurance checks, configuration boundaries, deterministic replay, input validation, and the public C ABI.

The included thresholds and scenarios are development examples only. Passing these software tests does not establish safety certification or authorize operation on a real vehicle.
