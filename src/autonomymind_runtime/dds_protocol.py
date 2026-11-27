"""Bounded reference wire protocol. Trusted network only, not an actuator interface."""
import json
import math
import uuid
from collections import OrderedDict

MAX_BYTES=1048576
def encode(message):
    data=json.dumps(message,allow_nan=False,separators=(",",":"))
    if len(data.encode("utf-8"))>MAX_BYTES:raise ValueError("message too large")
    return data

def decode(data):
    if len(data.encode("utf-8"))>MAX_BYTES:raise ValueError("message too large")
    msg=json.loads(data)
    if not isinstance(msg,dict) or msg.get("version")!=1:raise ValueError("invalid version")
    for key in ("id","sender","recipient","kind","correlation"):
        if not isinstance(msg.get(key),str) or not msg[key] or len(msg[key])>128:raise ValueError("invalid identity")
    for key in ("timestamp","expires"):
        v=msg.get(key)
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v):raise ValueError("invalid time")
    if not isinstance(msg.get("payload"),dict) or msg["expires"]<=msg["timestamp"]:raise ValueError("invalid payload")
    encode(msg)
    return msg

def envelope(sender,recipient,kind,payload,now,ttl=15,correlation=None):
    if not math.isfinite(ttl) or ttl<=0:raise ValueError("invalid TTL")
    i=uuid.uuid4().hex
    return dict(version=1,id=i,sender=sender,recipient=recipient,kind=kind,correlation=correlation or i,timestamp=now,expires=now+ttl,payload=payload)

class PeerDirectory:
    def __init__(self,lease=3):
        if not math.isfinite(lease) or lease<=0:raise ValueError("invalid lease")
        self.lease=lease;self.peers={}
    def observe(self,msg,wall,monotonic):
        p=msg["payload"]
        caps=p.get("capabilities")
        if msg["kind"]!="heartbeat" or msg["recipient"]!="*" or not isinstance(caps,list) or not all(isinstance(c,str) for c in caps):return False
        if not msg["timestamp"]<=wall+0.5 or wall>=msg["expires"]:return False
        old=self.peers.get(msg["sender"])
        if old and msg["timestamp"]<=old[0]:return False
        self.peers[msg["sender"]]=(msg["timestamp"],monotonic,tuple(caps))
        return True
    def discover(self,capability,monotonic):
        return sorted(k for k,(_,t,c) in self.peers.items() if 0<=monotonic-t<self.lease and capability in c)

class Receiver:
    def __init__(self,id,accepted,max_seen=4096):
        self.id=id;self.accepted=accepted;self.seen=OrderedDict();self.max_seen=max_seen
    def check(self,msg,now):
        if msg["recipient"]!=self.id:return "other_recipient"
        if msg["kind"] not in self.accepted:return "contract_mismatch"
        if msg["timestamp"]>now+0.5 or now>=msg["expires"]:return "expired"
        key=(msg["sender"],msg["id"])
        for k,t in list(self.seen.items()):
            if t<=now:del self.seen[k]
        if key in self.seen:return "duplicate"
        if len(self.seen)>=self.max_seen:return "capacity"
        self.seen[key]=msg["expires"]
        return "accepted"
