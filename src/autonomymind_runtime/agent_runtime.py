"""Transport-neutral agent kernel. In-process reference transport; no actuator authority."""
from dataclasses import dataclass
import json
import math
import copy

@dataclass(frozen=True)
class AgentSpec:
    id:str
    role:str
    version:str
    capabilities:tuple
    accepts:tuple
    emits:tuple

@dataclass(frozen=True)
class Envelope:
    id:str
    sender:str
    recipient:str
    kind:str
    timestamp:float
    expires_at:float
    payload:dict
    correlation_id:str
    schema_version:str="1.0"

class Runtime:
    def __init__(self,max_messages=10000):
        self.specs={};self.handlers={};self.health={};self.queue=[];self.seen=set()
        self.trace=[];self.counter=0;self.now=0.;self.max_messages=max_messages
    def register(self,spec,handler,lease_s):
        if not spec.id or not spec.version or not spec.role or spec.id in self.specs or not callable(handler) or not math.isfinite(lease_s) or lease_s<=0:
            raise ValueError("invalid agent registration")
        self.specs[spec.id]=spec;self.handlers[spec.id]=handler
        self.health[spec.id]=(self.now+lease_s,True)
    def heartbeat(self,id,lease_s):
        if id not in self.specs or not math.isfinite(lease_s) or lease_s<=0:raise ValueError("invalid heartbeat")
        self.health[id]=(self.now+lease_s,True)
    def advance(self,now):
        if not math.isfinite(now) or now<self.now:raise ValueError("invalid runtime clock")
        self.now=now
    def active(self,id):
        return id in self.health and self.health[id][1] and self.now<self.health[id][0]
    def discover(self,capability):
        return sorted(k for k,s in self.specs.items() if self.active(k) and capability in s.capabilities)
    def send(self,sender,recipient,kind,payload,ttl_s,correlation_id=""):
        if not math.isfinite(ttl_s) or ttl_s<=0:raise ValueError("invalid ttl")
        self.counter+=1
        e=Envelope(str(self.counter),sender,recipient,kind,self.now,self.now+ttl_s,copy.deepcopy(payload),correlation_id or str(self.counter))
        self.enqueue(e)
        return e.id
    def enqueue(self,e):
        if len(self.queue)>=self.max_messages:raise RuntimeError("queue limit")
        if not isinstance(e.payload,dict):raise ValueError("payload must be object")
        json.dumps(e.payload,allow_nan=False)
        self.queue.append(copy.deepcopy(e))
    def drain(self):
        count=0
        while self.queue:
            count+=1
            if count>self.max_messages:raise RuntimeError("delivery budget exceeded")
            e=self.queue.pop(0)
            reason=None
            if e.schema_version!="1.0" or not e.id or not e.correlation_id or not all(math.isfinite(x) for x in (e.timestamp,e.expires_at)) or not e.timestamp<=self.now<e.expires_at:reason="invalid_or_expired"
            elif (e.sender,e.id) in self.seen:reason="duplicate"
            elif not self.active(e.sender) or not self.active(e.recipient):reason="inactive_agent"
            elif e.kind not in self.specs[e.sender].emits or e.kind not in self.specs[e.recipient].accepts:reason="contract_mismatch"
            if reason:
                self.trace.append({"id":e.id,"kind":e.kind,"status":reason,"t":self.now});continue
            self.seen.add((e.sender,e.id))
            try:
                # A handler can emit only as its own identity using this scoped callback.
                def emit(recipient,kind,payload,ttl_s):
                    return self.send(e.recipient,recipient,kind,payload,ttl_s,e.correlation_id)
                self.handlers[e.recipient](copy.deepcopy(e),emit)
                self.trace.append({"id":e.id,"kind":e.kind,"status":"delivered","t":self.now,"correlation_id":e.correlation_id})
            except Exception as exc:
                self.health[e.recipient]=(self.now,False)
                self.trace.append({"id":e.id,"kind":e.kind,"status":"handler_failed","error":type(exc).__name__,"t":self.now})
