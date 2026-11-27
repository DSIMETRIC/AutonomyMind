from dataclasses import dataclass, field, asdict
from typing import Any
import json
import uuid

SCHEMA_VERSION = "1.0"

def _id(prefix: str) -> str:
    return f"{prefix}-{uuid.uuid4().hex[:12]}"

@dataclass
class Observation:
    observation_id: str
    agent_id: str
    agent_version: str
    timestamp_s: float
    frame_id: str
    source: str
    kind: str
    confidence: float
    position_m: list[float] = field(default_factory=lambda:[0.0,0.0,0.0])
    velocity_mps: list[float] = field(default_factory=lambda:[0.0,0.0,0.0])
    label: str = ""
    attributes: dict[str, Any] = field(default_factory=dict)
    provenance: dict[str, Any] = field(default_factory=dict)
    schema_version: str = SCHEMA_VERSION

    def to_dict(self): return asdict(self)

@dataclass
class WorldState:
    world_state_id: str
    timestamp_s: float
    frame_id: str
    producer_agent_id: str
    observations: list[Observation]
    ego: dict[str, Any] = field(default_factory=dict)
    teammates: list[dict[str, Any]] = field(default_factory=list)
    mission_context: dict[str, Any] = field(default_factory=dict)
    map_context: dict[str, Any] = field(default_factory=dict)
    provenance: dict[str, Any] = field(default_factory=dict)
    schema_version: str = SCHEMA_VERSION

    def to_dict(self):
        d=asdict(self); return d

@dataclass
class AgentProposal:
    proposal_id: str
    agent_id: str
    agent_version: str
    timestamp_s: float
    proposal_type: str
    target_robot_id: str
    confidence: float
    priority: int
    command: dict[str, Any]
    evidence: list[str]
    explanation: str
    model_version: str = "reference"
    policy_version: str = "reference-v1"
    requires_safety_validation: bool = True
    schema_version: str = SCHEMA_VERSION

    def to_dict(self): return asdict(self)

@dataclass
class SafetyCheck:
    name: str
    passed: bool
    measured: float = 0.0
    limit: float = 0.0
    details: str = ""

@dataclass
class SafetyDecision:
    decision_id: str
    proposal_id: str
    timestamp_s: float
    validator_id: str
    accepted: bool
    status: str
    reason: str
    checks: list[SafetyCheck]
    policy_version: str = "reference-safety-v1"
    config_sha256: str = "development-example"
    deterministic: bool = True
    schema_version: str = SCHEMA_VERSION

    def to_dict(self): return asdict(self)

def dumps(value: Any) -> str:
    if hasattr(value, "to_dict"): value=value.to_dict()
    return json.dumps(value, indent=2, sort_keys=True)

def new_id(prefix: str) -> str: return _id(prefix)
