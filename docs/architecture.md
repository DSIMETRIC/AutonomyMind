# AutonomyMind Architecture

AutonomyMind is organized as an agentic autonomy stack with deterministic safety boundaries, simulation, and semantic-data feedback.

## 1. System architecture

```mermaid
flowchart TB
  Sensors[Real / Simulated Sensors] --> PA[Perception Agent]
  PA --> O[Camera / LiDAR Observations]
  O --> SF[Temporal + Geometric Sensor Fusion]
  SF --> FO[Fused 3D Observation + Uncertainty]
  FO --> WM[World Model Agent]
  WM --> WS[WorldState]
  WS --> PLA[Planning Agent]
  WS --> CA[Coordination Agent]
  PLA --> AP[AgentProposal]
  CA --> AP
  AP --> DA[Decision Agent]
  DA --> SB[Deterministic Safety / RTA]
  SB --> SD[SafetyDecision]
  SD --> Robot[Drone / Rover / Robot]

  Isaac[Isaac Sim] --> Sensors
  Robot --> Telemetry[Telemetry / Logs]
  Telemetry --> Eval[Evaluation]
  Isaac --> Eval
  Eval --> Semantic[Semantic Annotation]
  Semantic --> Dataset[Versioned Dataset]
  Dataset --> Train[Train / Adapt]
  Train --> PA
```

## 2. Agent contract flow

```mermaid
sequenceDiagram
  participant Sim as Isaac / Sensor
  participant P as Perception Agent
  participant W as World Model
  participant Pl as Planning Agent
  participant S as Deterministic Safety
  participant R as Robot
  participant A as Annotation Connector

  Sim->>P: frame / sensor data
  P->>W: Observation
  alt confidence below review threshold
    P->>A: annotation request
  end
  W->>Pl: WorldState
  Pl->>S: AgentProposal
  S-->>Pl: SafetyDecision + evidence
  alt accepted
    S->>R: validated action
  else rejected
    S->>R: hold / no command
  end
```

## 3. Safety boundary

```mermaid
flowchart LR
  subgraph AI["Learned / Agentic Domain"]
    P[Perception]
    F[Fusion]
    W[World Model]
    PL[Planning]
    C[Coordination]
    D[Decision Support]
  end
  subgraph Deterministic["Deterministic Authority"]
    V[Input Validation]
    RTA[Runtime Assurance]
    E[Execution Gate]
  end
  P --> F --> W --> PL --> D
  W --> C --> D
  D --> V --> RTA --> E
  E -->|accepted| ACT[Actuator Command]
  RTA -->|rejected| HOLD[Hold / Safe Fallback]
```

AI components can generate candidates, probabilities, semantic interpretations, explanations, and coordination proposals. They cannot bypass deterministic validation.

## 4. Simulation and learning loop

```mermaid
flowchart LR
  Scenario[Scenario Library] --> Isaac[Isaac Sim]
  Isaac --> Run[Autonomy Run]
  Run --> Metrics[Evaluation]
  Metrics --> Edge[Edge Cases / Failures]
  Edge --> Annotate[SemanticMap-compatible Annotation]
  Annotate --> Review[Human Review]
  Review --> Data[Dataset Version]
  Data --> Train[Training / Adaptation]
  Train --> Registry[Model Registry]
  Registry --> Gate[Regression Gate]
  Gate --> Run
```

## 5. Cooperative robotics

```mermaid
flowchart TB
  D1[Drone Local Agents] --> SWM[Shared World Model]
  D2[Drone Local Agents] --> SWM
  RV[Rover Local Agents] --> SWM
  RB[Robot Local Agents] --> SWM
  SWM --> COORD[Coordination Agent]
  COORD --> MISSION[Mission Agent]
  MISSION --> L1[Local Decision Agent A]
  MISSION --> L2[Local Decision Agent B]
  L1 --> S1[Local Deterministic Safety]
  L2 --> S2[Local Deterministic Safety]
```

Local safety does not depend on permanent connectivity to the shared coordination layer.

## 6. Deployment topology

```mermaid
flowchart LR
  subgraph Dev["Development / Simulation"]
    SIM[Isaac Sim Container]
    AM[AutonomyMind Runtime]
    TEST[Test Containers]
    SEM[Semantic Annotation Connector]
    SIM --> AM --> SEM
    TEST --> AM
  end
  subgraph Edge["Edge Robot: Orin / Thor"]
    ROS[ROS 2 / AutonomyMind]
    GPU[Accelerated Perception]
    SAFE[Deterministic Safety]
    ROS --> GPU
    GPU --> SAFE
  end
  Dev <-->|versioned contracts / datasets / models| Edge
```


## 7. Camera + LiDAR fusion

```mermaid
flowchart LR
  CAM[Camera] --> DET[2D Detector]
  DET --> COBS[Camera Observation<br/>bbox + class + confidence]
  LIDAR[LiDAR] --> LOBS[LiDAR Observation<br/>3D point / range]
  CAL[Intrinsics + Extrinsics] --> FUSE[Temporal + Geometric Association]
  COBS --> FUSE
  LOBS --> FUSE
  FUSE --> FOBS[Fused 3D Observation<br/>semantic label + covariance + provenance]
  FOBS --> WM[World Model]
```

The fusion reference implementation rejects temporally misaligned pairs, transforms LiDAR points into the camera frame, projects them with calibrated intrinsics, associates projected points with 2-D detections, and publishes 3-D observations with explicit uncertainty and provenance.


## 8. Temporal 3D tracking

```mermaid
flowchart LR
  F[Fused 3D Observations] --> T[3D Multi-Object Tracker]
  T --> ID[Persistent track_id]
  T --> V[Velocity estimate]
  T --> C[Propagated covariance]
  T --> P[Short-horizon prediction]
  ID --> WM[World Model]
  V --> WM
  C --> WM
  P --> WM
  WM --> PLAN[Planning / Decision]
```

The reference tracker uses deterministic nearest-neighbor association with constant-velocity prediction, semantic-label compatibility, configurable 3-D gating, one-to-one assignment, and bounded track lifetime. This keeps the public baseline auditable while preserving the contract needed for future probabilistic or learned tracking.


## 9. Dynamic avoidance milestone

```mermaid
flowchart LR
  TRACKS[Persistent 3D Tracks] --> WM[Dynamic World Model]
  WM --> CP[Closest Approach / TTC]
  CP --> GEN[Candidate Generator]
  GEN --> GO[GO]
  GEN --> B[BRAKE]
  GEN --> L[LEFT]
  GEN --> R[RIGHT]
  GO --> RTA[Deterministic Safety]
  B --> RTA
  L --> RTA
  R --> RTA
  RTA --> DEC[Decision Agent]
  DEC --> CMD[Validated Maneuver]
  DEC --> TRACE[Episode Trace / Replay]
```

The public reference planner evaluates predicted relative motion over a finite horizon, produces multiple transparent candidates, validates them independently, and records the complete decision evidence for replay.


## 10. Cooperative autonomy execution

```mermaid
flowchart TB
  D[Drone Local Autonomy] --> SW[Shared World Model]
  R[Rover Local Autonomy] --> SW
  T[Shared 3D Tracks] --> SW
  M[Mission Tasks] --> C[Coordination Agent]
  SW --> C
  C --> A[Capability + Distance Assignment]
  C --> X[Inter-Robot Conflict Prediction]
  A --> D
  A --> R
  X --> Y[Yield / Coordination Directives]
  Y --> D
  Y --> R
  D --> DS[Drone Deterministic Safety]
  R --> RS[Rover Deterministic Safety]
```

The shared coordination layer can recommend task allocation and deconfliction, but it does not bypass each vehicle's local deterministic safety authority.
