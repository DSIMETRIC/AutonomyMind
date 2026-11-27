from pathlib import Path
import json

class SemanticMapConnector:
    """File-based public connector reference; no private service dependency."""
    def __init__(self, outbox: str): self.outbox=Path(outbox)

    def maybe_export(self, frame: dict, observation: dict, threshold: float=0.70):
        if float(observation.get("confidence",1.0)) >= threshold: return None
        self.outbox.mkdir(parents=True, exist_ok=True)
        payload={
            "schema_version":"1.0","request_type":"annotation",
            "frame":frame,"observation":observation,
            "reason":"low_confidence","review_status":"pending"
        }
        path=self.outbox / f"annotation-{observation['observation_id']}.json"
        path.write_text(json.dumps(payload,indent=2,sort_keys=True),encoding="utf-8")
        return str(path)
