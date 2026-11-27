"""Offline coordination reference: task offers are not motion authorizations."""
from dataclasses import dataclass
import math
from .fleet import Fleet, Member, Task

@dataclass(frozen=True)
class MissionTask:
    task: Task
    dependencies: tuple = ()

@dataclass(frozen=True)
class Offer:
    task_id: str
    member_id: str
    epoch: int
    expires_at: float

class MissionOrchestrator:
    def __init__(self,tasks,max_age_s,lease_s,ranker=None):
        self.tasks={t.task.id:t for t in tasks}
        if len(self.tasks)!=len(tasks) or not math.isfinite(lease_s) or lease_s<=0:
            raise ValueError("invalid mission")
        remaining=set(self.tasks);resolved=set()
        while remaining:
            ready={k for k in remaining if set(self.tasks[k].dependencies)<=resolved}
            if not ready:raise ValueError("missing or cyclic dependency")
            resolved.update(ready);remaining-=ready
        self.fleet=Fleet(max_age_s);self.lease_s=lease_s
        self.offers={};self.completed=set();self.epoch=0
        self.events=[];self.last_time=-math.inf;self.ranker=ranker
    def _time(self,now):
        if not math.isfinite(now) or now<self.last_time:raise ValueError("nonmonotonic event time")
        self.last_time=now
    def _event(self,now,kind,**data):
        self.events.append(dict(t=now,kind=kind,**data))
    def update(self,member,now):
        self._time(now)
        if member.timestamp>now:raise ValueError("future telemetry")
        return self.fleet.update(member)
    def reconcile(self,now):
        self._time(now)
        for key,o in list(self.offers.items()):
            m=self.fleet.members.get(o.member_id)
            if now>=o.expires_at or not m or not m.available or not self.fleet.fresh(m,now) or not self.tasks[key].task.required<=m.capabilities:
                del self.offers[key]
                self._event(now,"offer_revoked",task=key,member=o.member_id,epoch=o.epoch)
        busy={o.member_id for o in self.offers.values()}
        ready=[x.task for k,x in self.tasks.items() if k not in self.completed and k not in self.offers and set(x.dependencies)<=self.completed]
        available=Fleet(self.fleet.max_age_s)
        for m in self.fleet.members.values():
            if m.id not in busy:available.update(m)
        if self.ranker is None:
            assigned,_=available.allocate(ready,now)
        else:
            assigned={}
            for task in sorted(ready,key=lambda t:(-t.priority,t.id)):
                eligible=sorted((m for m in available.members.values() if m.available and available.fresh(m,now) and task.required<=m.capabilities),key=lambda m:m.id)
                if not eligible:continue
                request={"candidate_ids":[m.id for m in eligible],"features":{m.id:{"distance_m":math.dist(m.position,task.goal),"telemetry_age_s":now-m.timestamp,"uncertainty_m":m.uncertainty_m,"task_priority":task.priority} for m in eligible}}
                try:
                    advice=self.ranker(request)
                    ranked=advice["ranked_ids"]
                    ids=request["candidate_ids"]
                    if not isinstance(ranked,list) or len(ranked)!=len(ids) or set(ranked)!=set(ids) or advice.get("motion_authorized") is not False or advice.get("requires_safety_validation") is not True:
                        raise ValueError("invalid advisory result")
                    winner=ranked[0]
                    self._event(now,"model_advice",task=task.id,advice=advice)
                except Exception as exc:
                    self._event(now,"model_advice_rejected",task=task.id,error=type(exc).__name__)
                    continue
                assigned[task.id]=winner
                del available.members[winner]
        for k,m in assigned.items():
            self.epoch+=1
            o=Offer(k,m,self.epoch,now+self.lease_s)
            self.offers[k]=o
            self._event(now,"offer_issued",task=k,member=m,epoch=o.epoch,expires_at=o.expires_at)
        return dict(self.offers)
    def complete(self,task_id,member_id,epoch,now):
        self._time(now)
        o=self.offers.get(task_id)
        m=self.fleet.members.get(member_id)
        if not o or o.member_id!=member_id or o.epoch!=epoch or now>=o.expires_at or not m or not m.available or not self.fleet.fresh(m,now) or not self.tasks[task_id].task.required<=m.capabilities:
            self._event(now,"completion_rejected",task=task_id,member=member_id,epoch=epoch)
            return False
        del self.offers[task_id];self.completed.add(task_id)
        self._event(now,"task_completed",task=task_id,member=member_id,epoch=epoch)
        return True
    def snapshot(self,now,horizon_s,margin_m):
        self._time(now)
        return {"completed":sorted(self.completed),"offers":[o.__dict__ for o in self.offers.values()],
                "pending":sorted(set(self.tasks)-self.completed-set(self.offers)),
                **self.fleet.conflicts(now,horizon_s,margin_m)}
