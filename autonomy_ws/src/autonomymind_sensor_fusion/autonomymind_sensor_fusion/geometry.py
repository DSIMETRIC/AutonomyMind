import json
import math
from dataclasses import dataclass

@dataclass(frozen=True)
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float

@dataclass(frozen=True)
class Extrinsics:
    # lidar -> camera rigid transform, row-major rotation + translation.
    r00: float; r01: float; r02: float
    r10: float; r11: float; r12: float
    r20: float; r21: float; r22: float
    tx: float; ty: float; tz: float

    @staticmethod
    def identity():
        return Extrinsics(1,0,0,0,1,0,0,0,1,0,0,0)

def transform_point(p, ex: Extrinsics):
    x,y,z=[float(v) for v in p]
    return (
        ex.r00*x+ex.r01*y+ex.r02*z+ex.tx,
        ex.r10*x+ex.r11*y+ex.r12*z+ex.ty,
        ex.r20*x+ex.r21*y+ex.r22*z+ex.tz,
    )

def project_point(p_camera, cam: CameraModel):
    x,y,z=[float(v) for v in p_camera]
    if z <= 1e-9 or not all(math.isfinite(v) for v in (x,y,z)):
        return None
    return (cam.fx*x/z+cam.cx, cam.fy*y/z+cam.cy)

def point_in_bbox(uv, bbox_xyxy, margin_px=0.0):
    if uv is None:
        return False
    u,v=uv
    x1,y1,x2,y2=[float(x) for x in bbox_xyxy]
    m=float(margin_px)
    return (x1-m) <= u <= (x2+m) and (y1-m) <= v <= (y2+m)

def load_calibration(path):
    data=json.loads(open(path,"r",encoding="utf-8").read())
    c=data["camera"]
    e=data["lidar_to_camera"]
    cam=CameraModel(float(c["fx"]),float(c["fy"]),float(c["cx"]),float(c["cy"]))
    r=e["rotation_row_major"]; t=e["translation_m"]
    ex=Extrinsics(*[float(x) for x in r],*[float(x) for x in t])
    return cam,ex
