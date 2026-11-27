# Test strategy

AutonomyMind uses three test layers:

- **Unit** — individual agents, contracts, and deterministic safety behavior.
- **Integration** — agent-to-agent flow plus SemanticMap-compatible annotation export.
- **E2E** — complete simulated scenario from sensor frame to deterministic safety decision.

All reference tests run inside Docker with:

```bash
docker compose run --rm test
```

ROS 2 interface/core tests run in a separate ROS Jazzy image:

```bash
docker compose run --rm ros-test
```
