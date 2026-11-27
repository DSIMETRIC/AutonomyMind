import json
from autonomymind_interfaces.msg import Observation
from autonomymind_sensor_fusion.geometry import CameraModel, Extrinsics
from autonomymind_sensor_fusion.fusion_core import associate

def obs(oid,ts,kind,pos,label,conf,attrs):
    m=Observation()
    m.schema_version="1.0"; m.observation_id=oid; m.agent_id="t"; m.agent_version="1"
    m.timestamp_s=ts; m.frame_id="sensor"; m.source="test"; m.kind=kind; m.confidence=conf
    m.position_m=list(pos); m.velocity_mps=[0.0,0.0,0.0]; m.label=label
    m.attributes_json=json.dumps(attrs); m.provenance_json="{}"
    return m

def test_camera_lidar_association_and_uncertainty():
    cam=CameraModel(100,100,50,50)
    c=obs("c1",1.00,"detection",(0,0,0),"box",0.9,{"bbox_xyxy":[90,40,110,60]})
    l=obs("l1",1.03,"detection",(1,0,2),"nearest_obstacle",1.0,{"distance_m":2.23})
    a=associate([c],[l],cam,Extrinsics.identity(),max_dt_s=0.1,bbox_margin_px=0)
    assert len(a)==1
    assert a[0].label=="box"
    assert a[0].confidence>0.0
    assert all(v>0.0 for v in a[0].covariance_diag)

def test_rejects_bad_time_alignment():
    cam=CameraModel(100,100,50,50)
    c=obs("c1",1.0,"detection",(0,0,0),"box",0.9,{"bbox_xyxy":[90,40,110,60]})
    l=obs("l1",2.0,"detection",(1,0,2),"nearest_obstacle",1.0,{})
    assert associate([c],[l],cam,Extrinsics.identity(),max_dt_s=0.1)==[]
