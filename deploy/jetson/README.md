# Jetson Orin / Thor deployment

This overlay runs the AutonomyMind ROS adapters as arm64 containers on Jetson.

It intentionally does not vendor or rebuild NVIDIA Isaac ROS. Run the required Isaac ROS inference/driver containers according to NVIDIA's supported workflow and connect them through ROS 2 topics.

## Start adapters

```bash
docker compose -f deploy/jetson/compose.yaml build
docker compose -f deploy/jetson/compose.yaml up
```

Defaults:

- camera detections: `/detections`
- LiDAR scan: `/scan`
- normalized camera observations: `/autonomymind/observations/camera`
- normalized LiDAR observations: `/autonomymind/observations/lidar`

Override producer topics:

```bash
CAMERA_DETECTIONS_TOPIC=/my_detector/detections \
LIDAR_SCAN_TOPIC=/lidar/scan \
docker compose -f deploy/jetson/compose.yaml up
```

Host networking is used so DDS discovery can interoperate with separately managed Isaac ROS containers on the same Jetson.

Before deployment, validate the exact JetPack/Isaac ROS release combination supported by the target Orin or Thor system.
