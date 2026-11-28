"""Trusted local DDS advisory reservation service; no actuator interface."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String
from .dds_protocol import envelope,encode,decode,Receiver
from .spatiotemporal import Coordinator,Request

class ReservationAgent(Node):
    def __init__(self,config,output):
        super().__init__("agent_reservations")
        self.coordinator=Coordinator([r["member"] for r in config["requests"]],**config["settings"])
        self.lease=config["lease_s"];self.output=Path(output)
        self.receiver=Receiver("reservations",{"reservation.submit","reservation.query"})
        qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE)
        self.pub=self.create_publisher(String,"/autonomymind_agents/events",qos)
        self.sub=self.create_subscription(String,"/autonomymind_agents/events",self.receive,qos)
        self.timer=self.create_timer(.25,self.tick)
        self.last=None
    def send(self,m):self.pub.publish(String(data=encode(m)))
    def state(self):
        c=self.coordinator
        return {"revision":c.revision,"active":c.current is not None,"plan":None if c.current is None else {k:v for k,v in c.current.items() if k!="rejections"},"motion_authorized":False,"requires_safety_validation":True}
    def tick(self):
        try:
            now=time.time();self.coordinator.advance(now)
            self.send(envelope("reservations","*","heartbeat",{"capabilities":["trajectory_reservations"],"role":"coordination"},now,3))
            state=self.state()
            if state!=self.last:
                self.output.parent.mkdir(parents=True,exist_ok=True)
                tmp=self.output.with_suffix(".tmp")
                tmp.write_text(json.dumps(state,indent=2,allow_nan=False)+"\n")
                tmp.replace(self.output);self.last=state
        except Exception as exc:self.get_logger().error(str(exc))
    def receive(self,raw):
        try:
            m=decode(raw.data);now=time.time()
            if self.receiver.check(m,now)!="accepted":return
            self.coordinator.advance(now)
            if m["kind"]=="reservation.query":
                result=self.state()
            else:
                try:
                    p=m["payload"]
                    if p.get("frame_id")!="world_enu":raise ValueError("invalid frame")
                    # Bound the DDS reference batch and diagnostic payload.
                    rows=p["requests"]
                    if not isinstance(rows,list) or len(rows)>8 or any(len(r["points"])>32 for r in rows):
                        raise ValueError("DDS reference batch limits")
                    plan=self.coordinator.plan([Request(**r) for r in rows],now,p["expected_revision"],self.lease)
                    result={"status":plan["status"],"revision":plan["revision"],"initial_conflicts":plan["initial_conflicts"],"reservations":plan["reservations"],"rejected_delay_count":len(plan["rejections"]),"motion_authorized":False,"requires_safety_validation":True}
                except (ValueError,TypeError,KeyError) as exc:
                    result={"status":"rejected","reason":str(exc),"revision":self.coordinator.revision,"motion_authorized":False,"requires_safety_validation":True}
            self.send(envelope("reservations",m["sender"],"reservation.reply",result,time.time(),3,m["correlation"]))
        except Exception as exc:self.get_logger().warning("reservation event failed: "+str(exc))

def main():
    p=argparse.ArgumentParser();p.add_argument("--scenario",required=True);p.add_argument("--output",default="/outbox/reservation_state.json");a=p.parse_args()
    rclpy.init();n=ReservationAgent(json.loads(Path(a.scenario).read_text()),a.output)
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
