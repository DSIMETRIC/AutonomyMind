from dataclasses import dataclass, field

@dataclass
class DynamicTrack:
    track_id: str
    label: str
    position_m: list[float]
    velocity_mps: list[float]
    covariance_diag_m2: list[float]
    confidence: float
    predicted_position_m: list[float] = field(default_factory=list)

class DynamicWorldModel:
    def __init__(self):
        self.tracks={}

    def update_track(self, track: DynamicTrack):
        self.tracks[track.track_id]=track

    def snapshot(self, ego: dict, timestamp_s: float):
        return {
            "timestamp_s":float(timestamp_s),
            "ego":ego,
            "tracks":[
                {
                    "track_id":t.track_id,
                    "label":t.label,
                    "position_m":list(t.position_m),
                    "velocity_mps":list(t.velocity_mps),
                    "covariance_diag_m2":list(t.covariance_diag_m2),
                    "confidence":t.confidence,
                    "predicted_position_m":list(t.predicted_position_m),
                }
                for t in self.tracks.values()
            ],
        }
