# autonomymind_sensor_fusion

Camera-LiDAR temporal and geometric fusion for AutonomyMind.

## Inputs

- camera observations from `autonomymind_perception/camera_detection_adapter`;
- LiDAR observations from `autonomymind_perception/lidar_scan_adapter`.

## Output

`autonomymind_interfaces/msg/Observation` with:

- 3-D position from LiDAR;
- semantic class from camera detection;
- association score;
- diagonal covariance estimate;
- camera + LiDAR provenance;
- synchronized timestamp.

## Calibration

The fusion node requires a camera intrinsic matrix and a LiDAR-to-camera rigid transform in an external JSON calibration file.

The checked-in `config/example_calibration.json` is illustrative only and must not be used as real hardware calibration.

## Run

```bash
ros2 run autonomymind_sensor_fusion camera_lidar_fusion --ros-args \
  -p calibration_file:=/config/calibration.json \
  -p camera_topic:=/autonomymind/observations/camera \
  -p lidar_topic:=/autonomymind/observations/lidar \
  -p output_topic:=/autonomymind/observations/fused
```

## Current association model

1. Reject observations outside the configured time window.
2. Transform LiDAR points into the camera frame.
3. Project them with the configured camera intrinsics.
4. Require projected points to fall inside the camera bounding box.
5. Score candidates using temporal and image-space proximity.
6. Publish the best one-to-one association.

This is a deterministic reference fusion method. Future versions can replace the association model with learned or probabilistic fusion without changing the output contract.
