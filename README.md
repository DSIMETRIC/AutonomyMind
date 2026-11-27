# AutonomyMind

AutonomyMind is an open, agentic autonomy architecture for cooperative drones, rovers, and robotic systems.

The platform is organized around four pillars:

1. **Agentic Autonomy** — specialized AI agents for perception, fusion, world modeling, planning, coordination, and decision support.
2. **Simulation** — integrated simulation and synthetic-data workflows, with NVIDIA Isaac Sim as the primary simulation target.
3. **Semantic Data** — annotation, dataset, map, and model workflows through a clean SemanticMap-compatible integration boundary.
4. **Deterministic Safety** — operational safety and runtime-assurance checks remain outside unconstrained AI authority.

## Architecture

```text
Real / Synthetic Sensors
        |
        v
 Perception Agent
        |
        v
 Sensor Fusion Agent
        |
        v
 World Model / Semantic Memory
        |
        +--------------------+
        |                    |
        v                    v
 Planning Agent       Coordination Agent
        |                    |
        +---------+----------+
                  |
                  v
            Decision Agent
                  |
                  v
      Deterministic Safety / RTA
                  |
                  v
          Robot / Drone / Rover
```

The AI layer may reason, rank, propose, explain, and coordinate. Safety-critical execution remains constrained by deterministic policies and runtime-assurance gates.

## Closed-loop development

```text
Real Data
   |
   v
Semantic Annotation / Review
   |
   v
Datasets + Model Registry
   |
   v
Train / Adapt / Evaluate
   |
   v
Autonomy Agents
   |
   v
Decision + Safety
   |
   v
Robot
   ^
   |
Simulation / Isaac Sim
   |
   +--> Synthetic Data
   +--> Edge Cases
   +--> Regression Scenarios
   +--> Evaluation
```

Simulation failures and difficult edge cases can feed annotation and retraining workflows, creating a continuous improvement loop between data, simulation, and autonomy.

## Repository layout

```text
AutonomyMind/
├── agents/
│   ├── perception_agent/
│   ├── sensor_fusion_agent/
│   ├── world_model_agent/
│   ├── localization_agent/
│   ├── planning_agent/
│   ├── coordination_agent/
│   ├── decision_agent/
│   └── safety_agent/
│
├── simulation/
│   └── isaac/
│       ├── worlds/
│       ├── scenarios/
│       ├── robots/
│       ├── sensors/
│       ├── synthetic_data/
│       └── evaluation/
│
├── semantic_data/
│   ├── semanticmap_connector/
│   ├── annotation_schemas/
│   ├── datasets/
│   ├── model_registry/
│   └── evaluation/
│
├── orchestration/
│   ├── agent_registry/
│   ├── contracts/
│   ├── events/
│   └── observability/
│
└── autonomy_ws/
    └── src/
        ├── autonomymind_common/
        ├── autonomymind_perception/
        ├── autonomymind_sensor_fusion/
        ├── autonomymind_world_model/
        ├── autonomymind_localization/
        ├── autonomymind_navigation/
        ├── autonomymind_coordination/
        ├── autonomymind_mission/
        ├── autonomymind_decision/
        └── autonomymind_safety/
```

## Agent model

AutonomyMind uses specialized agents with narrow contracts rather than one monolithic model controlling the full stack.

- **Perception Agent** — converts raw sensor streams into detections, semantic features, free-space estimates, and structured observations.
- **Sensor Fusion Agent** — combines observations across cameras, LiDAR, radar, GNSS, IMU, and other sources.
- **World Model Agent** — maintains semantic state, tracks, map context, confidence, provenance, and shared spatial memory.
- **Localization Agent** — estimates platform state across GNSS, VIO, SLAM, and other localization sources.
- **Planning Agent** — proposes routes, local trajectories, task plans, and recovery options.
- **Coordination Agent** — coordinates heterogeneous robots and shared missions.
- **Decision Agent** — ranks operational options and produces explainable recommendations.
- **Safety Agent** — exposes deterministic safety evidence and interfaces to runtime-assurance logic; it does not replace the deterministic safety core.

## Simulation

`simulation/isaac/` is the integration surface for Isaac Sim scenarios, robots, sensors, synthetic data generation, and regression evaluation.

The simulation layer is intended to support:

- sensor simulation;
- multi-robot scenarios;
- synthetic perception datasets;
- failure injection;
- domain randomization;
- scenario replay;
- edge-case generation;
- autonomy regression testing.

Simulation assets and third-party components must retain their own licenses and redistribution terms.

## Semantic data and annotation

`semantic_data/` defines the public integration boundary for semantic annotation and dataset workflows.

The connector is intentionally decoupled from any private annotation platform implementation. It should exchange versioned contracts for:

- images and sensor frames;
- annotation requests;
- polygons, masks, tracks, and semantic geometry;
- review status;
- provenance;
- model version;
- dataset version;
- evaluation results.

No private repository code is vendored into AutonomyMind.

## Cooperative autonomy

AutonomyMind is designed for heterogeneous teams:

```text
Drone Agent Stack ----+
Rover Agent Stack ----+--> Shared World Model
Robot Agent Stack ----+          |
                                v
                         Coordination Agent
                                |
                                v
                           Mission Agent
                                |
                                v
                      Local Decision Agents
                                |
                                v
                     Deterministic Safety
```

Each platform can maintain local autonomy while contributing to a shared world model and coordinated mission state.

## Safety boundary

AI agents may:

- propose;
- rank;
- explain;
- predict;
- coordinate;
- generate candidate plans.

AI agents must not bypass deterministic runtime-assurance checks.

The current `autonomymind_decision` core remains the operational decision and evidence boundary for the public development baseline.

## Reference vertical

A first executable vertical is included:

```text
Synthetic / Isaac-compatible frame
        ↓
Perception Agent
        ↓
Observation
        ↓
Sensor Fusion Agent
        ↓
Fused Observation
        ↓
World Model Agent
        ↓
WorldState
        ↓
Planning Agent
        ↓
Planning Proposal
        ↓
Decision Agent
        ↓
AgentProposal
        ↓
Deterministic Safety
        ↓
SafetyDecision
        ↓
optional annotation request when confidence is low
```

Run it entirely in Docker:

```bash
docker compose run --rm contract-check
docker compose run --rm test
docker compose run --rm demo
docker compose run --rm dynamic-demo
docker compose run --rm cooperative-demo
docker compose run --rm ros-test
```

For WSL / Ubuntu amd64:

```bash
export AUTONOMYMIND_PLATFORM=linux/amd64
docker compose run --rm test
```

For Jetson Orin / Thor arm64:

```bash
export AUTONOMYMIND_PLATFORM=linux/arm64
docker compose run --rm test
```

Architecture diagrams: [docs/architecture.md](docs/architecture.md)  
Platform matrix: [docs/platforms.md](docs/platforms.md)  
Test strategy: [docs/testing.md](docs/testing.md)  
Isaac bridge: [simulation/isaac/BRIDGE.md](simulation/isaac/BRIDGE.md)  
Real perception: [docs/perception.md](docs/perception.md)  
Jetson deployment: [deploy/jetson/README.md](deploy/jetson/README.md)

## Status

AutonomyMind is under active development. The public reference vertical is intentionally simple and deterministic so contracts, Docker deployment, CI, simulation integration, and safety boundaries can evolve without hiding behavior behind a monolithic model.

## License

AutonomyMind is distributed under the Apache License 2.0. See `LICENSE` and `NOTICE`.


## v0.1 dynamic avoidance milestone

The executable v0.1 reference scenario demonstrates:

```text
moving 3D track
    ↓
dynamic world model
    ↓
closest-approach / TTC risk
    ↓
GO / BRAKE / LEFT / RIGHT candidates
    ↓
deterministic candidate safety checks
    ↓
decision ranking
    ↓
validated maneuver
    ↓
JSONL episode trace
```

Run:

```bash
docker compose run --rm dynamic-demo
```

The trace is written to `semantic_data/outbox/dynamic_trace.jsonl`.


## Cooperative autonomy milestone

The cooperative reference scenario demonstrates:

```text
Drone local state ----+
                      +--> Shared World Model
Rover local state ----+
          |
          +--> Shared tracks
          +--> Mission tasks
                    |
                    v
            Coordination Agent
          /                     \
 capability-based tasking   inter-robot conflict prediction
          \                     /
                    v
             coordination directives
```

Run:

```bash
docker compose run --rm cooperative-demo
```

The current public policy is deliberately transparent: task assignment is capability- and distance-based, while predicted robot-to-robot conflicts generate explicit yield directives. Local robot safety remains independent and deterministic.


## Demo animation

 > **Demo animation — not a live Isaac Sim capture.**  
> Shows the AutonomyMind flow: perception → tracking → world model → planning → decision → safety → coordination.

![AutonomyMind demo](docs/assets/autonomymind-demo-concept.gif)

## Mission dashboard demo

A presentation-ready deterministic demo combines the cooperative drone/rover scenario with dynamic obstacle avoidance, candidate safety evidence, task allocation, and replay.

Start the complete stack:

```bash
docker compose -f compose.demo.yaml up --build
```

Then open:

```text
http://localhost:8080
```

The mission container first writes a versioned JSONL trace. The dashboard then replays each mission step and shows:

- drone and rover state;
- shared tracks;
- task assignments;
- coordination conflicts/directives;
- GO / BRAKE / LEFT / RIGHT candidates;
- candidate priority and confidence;
- deterministic safety status;
- selected maneuver;
- raw trace evidence.

The checked-in scenario is deterministic and CI-friendly. Isaac Sim can replace the fixture as the upstream sensor/world source through the existing Isaac integration boundary without changing the trace or dashboard contract.
