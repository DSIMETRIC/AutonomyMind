"""DDS integration probe: synthetic full fleet proposals, no motion."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String
from .dds_protocol import envelope,encode,decode,PeerDirectory,Receiver
from .spatiotemporal import Request,conflict

def main():
    p=argparse.ArgumentParser();p.add_argument("--scenario",required=True);p.add_argument("--output",default="/outbox/reservation_probe.json");a=p.parse_args()
    s=json.loads(Path(a.scenario).read_text())
    rclpy.init();n=Node("reservation_probe");directory=PeerDirectory();receiver=Receiver("reservation_probe",{"reservation.reply"});replies={}
    qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE)
    pub=n.create_publisher(String,"/autonomymind_agents/events",qos)
    def receive(raw):
        try:m=decode(raw.data)
        except ValueError:return
        if m["kind"]=="heartbeat":directory.observe(m,time.time(),time.monotonic())
        elif m["sender"]=="reservations" and receiver.check(m,time.time())=="accepted":replies[m["correlation"]]=m["payload"]
    sub=n.create_subscription(String,"/autonomymind_agents/events",receive,qos)
    def wait(fn,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.05)
            if fn():return True
        return False
    def ask(kind,payload):
        m=envelope("reservation_probe","reservations",kind,payload,time.time(),5)
        pub.publish(String(data=encode(m)))
        if not wait(lambda:m["correlation"] in replies,5):raise RuntimeError("reservation reply timeout")
        return replies.pop(m["correlation"])
    def require(ok,message):
        if not ok:raise RuntimeError(message)
    def submit(rows,revision):
        return ask("reservation.submit",{"frame_id":"world_enu","expected_revision":revision,"requests":rows})
    def check(plan):
        rows=plan["reservations"]
        require(plan["status"]=="scheduled" and len(rows)==len(s["requests"]) and not plan["motion_authorized"],"not fully scheduled")
        for i,x in enumerate(rows):
            for y in rows[i+1:]:
                a,b=(Request(**{k:v for k,v in r.items() if k!="delay_s"}) for r in (x,y))
                require(conflict(a,x["delay_s"],b,y["delay_s"],s["settings"]["horizon_s"],s["settings"]["margin_m"]) is None,"remaining conflict")
    try:
        require(wait(lambda:"reservations" in directory.discover("trajectory_reservations",time.monotonic()),30),"service unavailable")
        state=ask("reservation.query",{})
        first=submit(s["requests"],state["revision"]);check(first)
        require(bool(first["initial_conflicts"]),"no initial conflict")
        stale=submit(s["requests"],state["revision"])
        require(stale["status"]=="rejected","stale revision accepted")
        revised=[{**r,"priority":s["revised_priorities"].get(r["member"],r["priority"])} for r in s["requests"]]
        second=submit(revised,first["revision"]);check(second)
        require([r["member"] for r in first["reservations"]]!=[r["member"] for r in second["reservations"]],"priority order unchanged")
        missing=submit(revised[:-1],second["revision"])
        require(missing["status"]=="rejected","incomplete batch accepted")
        wait(lambda:False,s["lease_s"]+.5)
        expired=ask("reservation.query",{})
        require(not expired["active"] and expired["revision"]>second["revision"],"expiry not observed")
        result={"source":"synthetic_batch_over_live_dds","members":len(s["requests"]),"initial_conflicts":len(first["initial_conflicts"]),"scheduled":len(first["reservations"]),"revised_scheduled":len(second["reservations"]),"remaining_conflicts":0,"stale_revision_rejected":True,"incomplete_batch_rejected":True,"expired":True,"motion_authorized":False,"cpp_rta_assessed":False}
        out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps({"metrics":result,"first":first,"second":second,"expired_state":expired},indent=2)+"\n")
        print(json.dumps(result,indent=2));print("PASS: DDS reservation batch, priority revision, stale and incomplete rejection, expiry")
    finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
