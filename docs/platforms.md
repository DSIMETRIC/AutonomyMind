# Docker deployment matrix

The reference stack is Docker-first and targets both CPU architectures used by development workstations and NVIDIA edge systems.

| Target | CPU architecture | Reference platform | AutonomyMind runtime | ROS 2 image | Isaac Sim |
| --- | --- | --- | --- | --- | --- |
| WSL 2 + Ubuntu | amd64 | Windows host + Docker/WSL | Supported | Supported | Use native/headless setup only after compatibility check; NVIDIA's Compose web viewer is not supported on WSL |
| Ubuntu workstation/server | amd64 | Ubuntu + NVIDIA GPU optional | Supported | Supported | Primary local/headless simulation target |
| Jetson Orin | arm64 | JetPack / Ubuntu | Supported | Supported | Container is multi-arch, but run NVIDIA compatibility checks; remote workstation simulation remains a valid deployment pattern |
| Jetson AGX Thor / T5000 / T4000 | arm64 | JetPack 7 / Ubuntu 24.04 | Supported | Supported | Container is multi-arch; validate GPU/system compatibility before enabling simulator workloads |

NVIDIA currently lists JetPack 7 support for both Orin and Thor families and publishes Ubuntu 24.04-based Jetson Linux for the current stack.

## Runtime

### amd64 / WSL / Ubuntu

```bash
export AUTONOMYMIND_PLATFORM=linux/amd64
docker compose build
docker compose run --rm test
docker compose run --rm demo
docker compose run --rm ros-test
```

### Jetson Orin / Thor

```bash
export AUTONOMYMIND_PLATFORM=linux/arm64
docker compose build
docker compose run --rm test
docker compose run --rm demo
docker compose run --rm ros-test
```

GPU-accelerated edge agents should be introduced as separate images/profiles so the deterministic reference runtime remains portable.

## Multi-architecture build verification

```bash
docker buildx build --platform linux/amd64,linux/arm64 -f Dockerfile .
docker buildx build --platform linux/amd64,linux/arm64 -f Dockerfile.ros .
```

## Isaac Sim

Isaac Sim is an external NVIDIA dependency and is not redistributed by AutonomyMind.

Use:

```bash
export ISAAC_SIM_IMAGE=<supported NVIDIA Isaac Sim container tag>
export ACCEPT_EULA=Y
docker compose -f simulation/isaac/compose.isaac.yaml run --rm isaac-compat
```

Only after compatibility passes:

```bash
docker compose -f simulation/isaac/compose.isaac.yaml up isaac-sim
```

The simulator and AutonomyMind communicate through mounted exchange data and, in later integrations, ROS 2/DDS contracts. This keeps the autonomy runtime independently testable on hardware where the full simulator is not desirable.

## References

- NVIDIA Isaac Sim container documentation: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html
- NVIDIA JetPack: https://developer.nvidia.com/embedded/jetpack
- NVIDIA JetPack downloads: https://developer.nvidia.com/embedded/jetpack/downloads
