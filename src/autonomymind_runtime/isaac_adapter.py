import json
from pathlib import Path

class IsaacScenarioAdapter:
    """Transport-neutral adapter for exported/synthetic Isaac scenario events."""
    def load(self, path: str) -> list[dict]:
        data=json.loads(Path(path).read_text(encoding="utf-8"))
        if data.get("schema_version")!="1.0": raise ValueError("unsupported scenario schema")
        frames=data.get("frames",[])
        if not isinstance(frames,list) or not frames: raise ValueError("scenario must contain frames")
        return frames
