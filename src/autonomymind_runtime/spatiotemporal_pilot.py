"""Config-driven offline reservation evaluation."""
import argparse
import json
from pathlib import Path
from .spatiotemporal import Coordinator,Request,conflict

def run(s):
    requests=[Request(**r) for r in s["requests"]]
    c=Coordinator([r.member for r in requests],**s["settings"])
    initial=c.plan(requests,0,0,s["lease_s"])
    revised=[Request(**{**r,"priority":s["revised_priorities"].get(r["member"],r["priority"])}) for r in s["requests"]]
    final=c.plan(revised,1,c.revision,s["lease_s"])
    def verify(plan):
        rows=plan["reservations"]
        return sum(conflict(Request(**{k:v for k,v in a.items() if k!="delay_s"}),a["delay_s"],Request(**{k:v for k,v in b.items() if k!="delay_s"}),b["delay_s"],c.horizon,c.margin) is not None for i,a in enumerate(rows) for b in rows[i+1:])
    remaining=[verify(p) for p in (initial,final)]
    c.advance(1+s["lease_s"])
    metrics={"members":len(requests),"initial_conflicts":len(initial["initial_conflicts"]),"scheduled":len(initial["reservations"]),"revised_scheduled":len(final["reservations"]),"remaining_conflicts":sum(remaining),"priority_order_changed":[r["member"] for r in initial["reservations"]]!=[r["member"] for r in final["reservations"]],"expired":c.current is None}
    return {"source":"offline_piecewise_linear_reference","metrics":metrics,"initial":initial,"revised":final,"cpp_rta_assessed":False,"motion_authorized":False}

def main():
    p=argparse.ArgumentParser();p.add_argument("--scenario",required=True);p.add_argument("--output",required=True);a=p.parse_args()
    r=run(json.loads(Path(a.scenario).read_text()));out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(r,indent=2,allow_nan=False)+"\n")
    print(json.dumps(r["metrics"],indent=2))
    m=r["metrics"]
    if not (m["initial_conflicts"]>0 and m["scheduled"]==m["revised_scheduled"]==m["members"] and m["remaining_conflicts"]==0 and m["priority_order_changed"] and m["expired"]):
        raise SystemExit("FAIL: reference scheduling acceptance")
    print("PASS: spatiotemporal reference scheduling, priority revision and expiry")
if __name__=="__main__":main()
