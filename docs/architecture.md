# AutonomyMind Architecture

AutonomyMind is organized as an agentic autonomy stack with deterministic safety boundaries, simulation, and semantic-data feedback.

## 1. System architecture

```mermaid
flowchart TB
  Sensors[Real / Simulated Sensors] --> PA[Perception Agent]
  PA --> O[Observation]
  O --> SF[Sensor Fusion Agent]
  SF --> WM[World Model Agent]
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
