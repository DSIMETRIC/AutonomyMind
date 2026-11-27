import json
import math
import uuid
from autonomymind_interfaces.msg import Observation

def observation_id(prefix: str) -> str:
    return f"{prefix}-{uuid.uuid4().hex[:12]}"

def finite(value: float) -> bool:
    return math.isfinite(float(value))

def make_observation(*, prefix: str, agent_id: str, timestamp_s: float, frame_id: str,
                     source: str, kind: str, confidence: float, position, velocity,
                     label: str, provenance: dict, attributes: dict) -> Observation:
    msg=Observation()
    msg.schema_version="1.0"
    msg.observation_id=observation_id(prefix)
    msg.agent_id=agent_id
    msg.agent_version="0.1.0"
    msg.timestamp_s=float(timestamp_s)
    msg.frame_id=frame_id or "unknown"
    msg.source=source
    msg.kind=kind
    msg.confidence=max(0.0,min(1.0,float(confidence)))
    msg.position_m=[float(x) for x in position]
    msg.velocity_mps=[float(x) for x in velocity]
    msg.label=label
    msg.provenance_json=json.dumps(provenance,sort_keys=True)
    msg.attributes_json=json.dumps(attributes,sort_keys=True)
    return msg
