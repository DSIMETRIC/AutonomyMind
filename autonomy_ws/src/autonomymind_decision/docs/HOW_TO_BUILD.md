# How to Build AutonomyMind Decision

## ROS / colcon

```bash
cd <autonomymind>/autonomy_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rm -rf build/autonomymind_decision install/autonomymind_decision
colcon build --packages-select autonomymind_decision \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

The build generates `generated_limits.h` from `config/interface.conf`.

## Standalone compiler validation

```bash
cd <autonomymind>/autonomy_ws/src/autonomymind_decision
bash tools/test_pipeline.sh
```

## Sanitizers

```bash
SANITIZE=1 bash tools/test_pipeline.sh
```

This enables AddressSanitizer and UndefinedBehaviorSanitizer.
