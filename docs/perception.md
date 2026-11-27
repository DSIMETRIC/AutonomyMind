# Real perception integration

AutonomyMind supports two complementary perception paths without coupling downstream autonomy to one model vendor.

## Path A — WSL / Ubuntu development

A Dockerized ONNX Runtime reference detector accepts YOLO-style detection models.

```bash
docker build -f Dockerfile.vision -t autonomymind/vision:cpu .
docker run --rm \
  -v "$PWD/models:/models:ro" \
  -v "$PWD/data:/data:ro" \
  autonomymind/vision:cpu \
  --model /models/detector.onnx \
  --image /data/frame.jpg \
  --labels /models/labels.txt
```

The model is deliberately external to the repository. Users are responsible for the model license and redistribution rights.

The output is a JSON array conforming to the logical `Observation` contract.

## Path B — Jetson Orin / Thor

For NVIDIA edge deployment, the preferred path is:

```text
Camera
  -> Isaac ROS / TensorRT / NITROS detector
  -> vision_msgs/Detection2DArray
  -> autonomymind_perception camera_detection_adapter
  -> Observation
  -> Fusion / World Model / Planning / Decision / Safety
```

LiDAR:

```text
LiDAR driver / Isaac ROS
  -> sensor_msgs/LaserScan
  -> autonomymind_perception lidar_scan_adapter
  -> Observation
```

This means the autonomy contracts stay unchanged while inference can move from CPU development to GPU edge deployment.

## Why this split

The portable AutonomyMind core should not require CUDA, TensorRT, or a specific neural network. Hardware acceleration belongs in replaceable perception producers. Downstream agents consume versioned observations with provenance.

NVIDIA currently positions Isaac ROS as the GPU-accelerated ROS 2 foundation for Jetson and workstation deployment, including perception, localization and AI inference. NITROS is intended to reduce data-movement overhead in accelerated ROS graphs.

## Current limitations

- The CPU ONNX adapter currently supports YOLO-style detection outputs with box coordinates followed by class scores.
- Depth is not inferred from a 2-D bounding box.
- Camera and LiDAR observations are not yet time-synchronized in the ROS adapter layer.
- Real multi-sensor calibration and track association are the next implementation milestone.
