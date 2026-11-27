"""Bounded shared evidence store. Not a source of motion authorization."""
import copy
import math
from .dds_protocol import encode

def number(x):
    return isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x)

def vector(x,n):
    return isinstance(x,list) and len(x)==n and all(number(v) for v in x)

class SharedWorld:
    def __init__(self,frame="world_enu",max_age=5,capacity=256):
        if not frame or not number(max_age) or max_age<=0 or not isinstance(capacity,int) or not 1<=capacity<=256:
            raise ValueError("invalid world settings")
        self.frame=frame;self.max_age=max_age;self.capacity=capacity;self.records={};self.revision=0
    def expire(self,now):
        if not number(now):raise ValueError("invalid time")
        for key,r in list(self.records.items()):
            if now>=r["expires"]:
                del self.records[key];self.revision+=1
    def ingest(self,m,now):
        self.expire(now)
        if m["recipient"]!="*" or m["kind"] not in ("world.observation","scene.semantic","world.member"):
            return "unsupported"
        if not number(m["timestamp"]) or not number(m["expires"]) or not m["timestamp"]<=now+.5 or now>=m["expires"] or now-m["timestamp"]>=self.max_age:
            return "stale"
        p=m["payload"];kind=m["kind"]
        if len(encode(p).encode())>2048:return "oversize"
        if kind=="scene.semantic":
            if p.get("kind")!="unverified_semantic_description" or p.get("metric_geometry_available") is not False or p.get("motion_authorized") is not False or p.get("requires_independent_validation") is not True:
                return "invalid_semantic"
            d=p.get("description")
            if not isinstance(d,dict) or set(d)!={"objects","hazards","summary"} or not isinstance(d["summary"],str) or not all(isinstance(d[k],list) and all(isinstance(v,str) for v in d[k]) for k in ("objects","hazards")):
                return "invalid_semantic"
            identity=p.get("image_sha256")
            if not isinstance(identity,str) or len(identity)!=64 or any(c not in "0123456789abcdef" for c in identity):return "invalid_identity"
            stamp=m["timestamp"]
        else:
            identity=p.get("observation_id" if kind=="world.observation" else "member_id")
            stamp=p.get("timestamp_s")
            if not isinstance(identity,str) or not identity or len(identity)>128:return "invalid_identity"
            if p.get("frame_id")!=self.frame:return "frame_mismatch"
            if not number(stamp) or stamp>m["timestamp"]+.5 or now-stamp>=self.max_age:return "stale_source"
            if not vector(p.get("position_m"),3) or not vector(p.get("velocity_mps"),3):return "invalid_geometry"
            u=p.get("uncertainty_m")
            if not number(u) or u<0:return "invalid_uncertainty"
            if kind=="world.observation":
                c=p.get("confidence")
                if not number(c) or not 0<=c<=1 or not isinstance(p.get("label"),str):return "invalid_observation"
            elif not isinstance(p.get("available"),bool):return "invalid_member"
        key=(kind,m["sender"],identity)
        previous=self.records.get(key)
        if previous and stamp<=previous["source_time"]:return "duplicate_or_old"
        if not previous and len(self.records)>=self.capacity:return "capacity"
        self.records[key]={"kind":kind,"sender":m["sender"],"identity":identity,"source_time":stamp,
                           "expires":min(m["expires"],stamp+self.max_age,m["timestamp"]+self.max_age),
                           "event_id":m["id"],"payload":copy.deepcopy(p)}
        self.revision+=1
        return "accepted"
    def snapshot(self,now):
        self.expire(now)
        groups={"world.observation":[],"scene.semantic":[],"world.member":[]}
        for key,r in sorted(self.records.items()):groups[key[0]].append(copy.deepcopy(r))
        return {"schema_version":"1.0","frame_id":self.frame,"timestamp_s":now,"revision":self.revision,
                "observations":groups["world.observation"],"semantic_reports":groups["scene.semantic"],"members":groups["world.member"],
                "coverage_complete":False,"motion_authorized":False}
