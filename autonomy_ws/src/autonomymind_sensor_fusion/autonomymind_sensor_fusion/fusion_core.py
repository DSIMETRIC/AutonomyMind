import json
import math
from dataclasses import dataclass
from autonomymind_interfaces.msg import Observation
from .geometry import CameraModel, Extrinsics, transform_point, project_point, point_in_bbox

@dataclass
class Association:
    camera_id: str
    lidar_id: str
    score: float
    position_m: tuple[float,float,float]
    covariance_diag: tuple[float,float,float]
    label: str
    confidence: float

def _attrs(msg: Observation):
    try:
        return json.loads(msg.attributes_json or "{}")
    except Exception:
        return {}

def _prov(msg: Observation):
    try:
        return json.loads(msg.provenance_json or "{}")
    except Exception:
        return {}

def associate(camera_obs, lidar_obs, cam: CameraModel, ex: Extrinsics,
              max_dt_s=0.10, bbox_margin_px=8.0, range_sigma_m=0.15, bearing_sigma_rad=0.01):
    out=[]
    used=set()
    for c in camera_obs:
        ca=_attrs(c)
        bbox=ca.get("bbox_xyxy")
        if not bbox:
            continue
        best=None
        for li,l in enumerate(lidar_obs):
            if li in used:
                continue
            dt=abs(float(c.timestamp_s)-float(l.timestamp_s))
            if dt > max_dt_s:
                continue
            lp=tuple(float(v) for v in l.position_m)
            cp=transform_point(lp,ex)
            uv=project_point(cp,cam)
            if not point_in_bbox(uv,bbox,bbox_margin_px):
                continue
            cx=(float(bbox[0])+float(bbox[2]))*0.5
            cy=(float(bbox[1])+float(bbox[3]))*0.5
            du=(uv[0]-cx)/(max(1.0,float(bbox[2])-float(bbox[0])))
            dv=(uv[1]-cy)/(max(1.0,float(bbox[3])-float(bbox[1])))
            spatial=math.sqrt(du*du+dv*dv)
            score=max(0.0,1.0-spatial)*max(0.0,1.0-dt/max_dt_s)
            if best is None or score > best[0]:
                best=(score,li,l,lp)
        if best is None:
            continue
        score,li,l,lp=best
        used.add(li)
        distance=math.sqrt(sum(v*v for v in lp))
        sigma_xy=max(range_sigma_m,abs(distance*bearing_sigma_rad))
        cov=(sigma_xy*sigma_xy,sigma_xy*sigma_xy,range_sigma_m*range_sigma_m)
        confidence=max(0.0,min(1.0,float(c.confidence)*float(l.confidence)*score))
        out.append(Association(
            camera_id=c.observation_id,
            lidar_id=l.observation_id,
            score=score,
            position_m=lp,
            covariance_diag=cov,
            label=c.label or "object",
            confidence=confidence,
        ))
    return out
