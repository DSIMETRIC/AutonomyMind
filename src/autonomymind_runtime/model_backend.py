"""Model-independent advisory inference contract; no motion authorization."""
from dataclasses import dataclass
from typing import Protocol
import json
import math
import copy
import hashlib

@dataclass(frozen=True)
class ModelSpec:
    id: str
    version: str
    backend: str
    artifact_sha256: str

class Backend(Protocol):
    def infer(self, request: dict) -> dict: ...

class ModelRegistry:
    def __init__(self):
        self._models = {}
    def register(self, spec: ModelSpec, backend: Backend):
        if not all(isinstance(s,str) and s for s in (spec.id,spec.version,spec.backend)):
            raise ValueError("invalid model identity")
        if len(spec.artifact_sha256)!=64 or any(c not in "0123456789abcdef" for c in spec.artifact_sha256):
            raise ValueError("invalid artifact digest")
        key=(spec.id,spec.version)
        if key in self._models or not callable(getattr(backend,"infer",None)):
            raise ValueError("invalid or duplicate backend")
        self._models[key]=(spec,backend)
    def advise(self, model_id, version, request):
        spec,backend=self._models[(model_id,version)]
        # Canonical serialization rejects non-finite numbers and detached inputs.
        raw=json.dumps(request,sort_keys=True,allow_nan=False,separators=(",",":"))
        clean=json.loads(raw)
        if not isinstance(clean,dict) or not isinstance(clean.get("candidate_ids"),list):
            raise ValueError("candidate IDs required")
        ids=clean["candidate_ids"]
        if not ids or not all(isinstance(x,str) and x for x in ids) or len(set(ids))!=len(ids):
            raise ValueError("invalid candidate IDs")
        result=backend.infer(copy.deepcopy(clean))
        if not isinstance(result,dict) or set(result)!={"ranked_ids","explanation"}:
            raise ValueError("unexpected advisory fields")
        ranked=result["ranked_ids"]; explanation=result["explanation"]
        if not isinstance(ranked,list) or not all(isinstance(x,str) for x in ranked) or len(ranked)!=len(ids) or set(ranked)!=set(ids):
            raise ValueError("rank must be a permutation of supplied candidates")
        if not isinstance(explanation,str) or len(explanation)>4096:
            raise ValueError("invalid explanation")
        return {"ranked_ids":list(ranked),"explanation":explanation,
                "model_id":spec.id,"model_version":spec.version,"backend":spec.backend,
                "artifact_sha256":spec.artifact_sha256,
                "input_sha256":hashlib.sha256(raw.encode()).hexdigest(),
                "requires_safety_validation":True,"motion_authorized":False}

class ReferenceRanker:
    """Deterministic baseline, not learned inference."""
    def infer(self,request):
        ids=request["candidate_ids"];scores=request.get("scores",{})
        if not isinstance(scores,dict) or set(scores)!=set(ids) or not all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) for x in scores.values()):
            raise ValueError("invalid scores")
        return {"ranked_ids":sorted(ids,key=lambda i:(-scores[i],i)),
                "explanation":"Reference ordering by supplied scores; not a safety assessment."}
