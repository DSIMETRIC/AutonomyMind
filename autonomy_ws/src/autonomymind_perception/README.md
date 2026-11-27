# autonomymind_perception

ROS 2 adapters that normalize real perception outputs into the AutonomyMind `Observation` contract.

## Camera detector adapter

Input:

`vision_msgs/msg/Detection2DArray`

Output:

`autonomymind_interfaces/msg/Observation`

This intentionally decouples AutonomyMind from a specific detector. Any camera model or accelerated pipeline that publishes standard ROS detections can be connected without changing downstream agents.

Examples include:

- CPU/GPU detector running on an Ubuntu or WSL development machine;
- TensorRT inference on an NVIDIA workstation;
- Isaac ROS / NITROS accelerated pipelines on Jetson Orin or Thor.

Run:

```bash
ros2 run autonomymind_perception camera_detection_adapter --ros-args \
  -p input_topic:=/detections \
  -p output_topic:=/autonomymind/observations/camera
```

## LiDAR adapter

Input:

`sensor_msgs/msg/LaserScan`

Output:

`autonomymind_interfaces/msg/Observation`

The reference implementation publishes the nearest valid obstacle with range and bearing evidence.

```bash
ros2 run autonomymind_perception lidar_scan_adapter --ros-args \
  -p input_topic:=/scan \
  -p output_topic:=/autonomymind/observations/lidar
```

## Safety and semantics

Perception confidence is evidence, not safety authority. Observations still flow through fusion, world modeling, planning/decision, and the deterministic safety boundary.

No model weights are stored in this repository.
