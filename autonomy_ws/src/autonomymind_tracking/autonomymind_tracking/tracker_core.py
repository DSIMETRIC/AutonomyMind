import math
from dataclasses import dataclass, field

@dataclass
class Detection3D:
    observation_id: str
    timestamp_s: float
    label: str
    confidence: float
    position_m: tuple[float,float,float]
    covariance_diag_m2: tuple[float,float,float]

@dataclass
class Track:
    track_id: str
    label: str
    confidence: float
    timestamp_s: float
    position_m: tuple[float,float,float]
    velocity_mps: tuple[float,float,float]
    covariance_diag_m2: tuple[float,float,float]
    age: int = 1
    missed: int = 0
    source_ids: list[str] = field(default_factory=list)

    def predict(self, t: float):
        dt=max(0.0,float(t)-self.timestamp_s)
        return tuple(self.position_m[i]+self.velocity_mps[i]*dt for i in range(3))

class MultiObjectTracker:
    def __init__(self, gate_m=2.0, max_missed=5, process_noise_m2_s=0.25, velocity_alpha=0.6):
        self.gate_m=float(gate_m)
        self.max_missed=int(max_missed)
        self.process_noise=float(process_noise_m2_s)
        self.velocity_alpha=float(velocity_alpha)
        self.tracks={}
        self._counter=0

    def _new_id(self):
        self._counter += 1
        return f"track-{self._counter:06d}"

    @staticmethod
    def _dist(a,b):
        return math.sqrt(sum((float(a[i])-float(b[i]))**2 for i in range(3)))

    def update(self, detections: list[Detection3D], timestamp_s: float):
        timestamp_s=float(timestamp_s)
        unmatched_tracks=set(self.tracks.keys())
        unmatched_det=set(range(len(detections)))
        pairs=[]
        candidates=[]
        for tid,t in self.tracks.items():
            pred=t.predict(timestamp_s)
            for i,d in enumerate(detections):
                if t.label and d.label and t.label != d.label:
                    continue
                dist=self._dist(pred,d.position_m)
                if dist <= self.gate_m:
                    candidates.append((dist,tid,i))
        candidates.sort()
        for dist,tid,i in candidates:
            if tid not in unmatched_tracks or i not in unmatched_det:
                continue
            unmatched_tracks.remove(tid); unmatched_det.remove(i)
            pairs.append((tid,i))

        for tid,i in pairs:
            t=self.tracks[tid]; d=detections[i]
            dt=max(1e-6,d.timestamp_s-t.timestamp_s)
            measured_v=tuple((d.position_m[k]-t.position_m[k])/dt for k in range(3))
            a=self.velocity_alpha
            velocity=tuple(a*measured_v[k]+(1.0-a)*t.velocity_mps[k] for k in range(3))
            covariance=tuple(max(1e-9,float(d.covariance_diag_m2[k])+self.process_noise*dt) for k in range(3))
            self.tracks[tid]=Track(
                track_id=tid,label=d.label,confidence=max(t.confidence,d.confidence),
                timestamp_s=d.timestamp_s,position_m=d.position_m,velocity_mps=velocity,
                covariance_diag_m2=covariance,age=t.age+1,missed=0,
                source_ids=(t.source_ids+[d.observation_id])[-20:],
            )

        for i in unmatched_det:
            d=detections[i]; tid=self._new_id()
            self.tracks[tid]=Track(
                track_id=tid,label=d.label,confidence=d.confidence,timestamp_s=d.timestamp_s,
                position_m=d.position_m,velocity_mps=(0.0,0.0,0.0),
                covariance_diag_m2=d.covariance_diag_m2,age=1,missed=0,source_ids=[d.observation_id],
            )

        for tid in list(unmatched_tracks):
            t=self.tracks[tid]
            dt=max(0.0,timestamp_s-t.timestamp_s)
            cov=tuple(v+self.process_noise*dt for v in t.covariance_diag_m2)
            self.tracks[tid]=Track(**{**t.__dict__,"missed":t.missed+1,"covariance_diag_m2":cov})
            if self.tracks[tid].missed > self.max_missed:
                del self.tracks[tid]

        return list(self.tracks.values())
