# Closed-loop simulation boundary

This milestone provides reference point-clearance authorization, not a certified runtime assurance implementation. No actuator interface is enabled. The existing JSONL exchange is offline, not a live ROS bridge.

Next integration acceptance criteria:
- Explicit simulator version and compatible GPU.
- USD scene with physical robot models, calibrated camera/LiDAR and ROS 2 clock/odometry.
- Time-aligned tracks in a common frame with freshness and source health.
- Independent C++ RTA integration and vehicle-specific dynamics, braking distance, uncertainty and communication watchdog.
- Simulator-only command adapter with explicit opt-in and no physical robot routing.
- SIL scenarios covering collision, sensor dropout, stale state, no accepted candidate and recovery.
- Real dashboard trace and recorded simulator capture.

The Python reference gate assumes point geometry and instantaneous constant velocity, with an illustrative five-second horizon and two-metre clearance. Its acceptance is not evidence that a real maneuver is safe.
