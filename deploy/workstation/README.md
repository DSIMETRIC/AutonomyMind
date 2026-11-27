# WSL / Ubuntu workstation

The portable reference stack runs with the root `compose.yaml`.

```bash
export AUTONOMYMIND_PLATFORM=linux/amd64
docker compose run --rm test
docker compose run --rm ros-test
docker compose run --rm demo
```

For direct image inference with a user-supplied YOLO-style ONNX model:

```bash
docker compose -f compose.vision.yaml build
docker compose -f compose.vision.yaml run --rm vision-test
docker compose -f compose.vision.yaml run --rm vision-cpu
```

Expected mounts:

- `models/detector.onnx`
- `models/labels.txt`
- `data/frame.jpg`

Model weights are intentionally excluded from Git.
