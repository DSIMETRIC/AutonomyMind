"""ROS 2 reference worker processes for offline mission replay only."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from .dds_protocol import envelope,encode,decode,PeerDirectory,Receiver
from .industrial_pilot import run

ROLES={"mission":(["mission"],{"mission.submit"}),"coordination":(["allocate"],{"mission.request"}),"evaluation":(["audit"],{"mission.report"})}

class Worker(Node):
    def __init__(self,id,role,output,ranker=None):
        super().__init__("agent_"+id)
        self.id=id;self.role=role;self.output=Path(output);self.ranker=ranker
        caps,accepted=ROLES[role];self.caps=caps
        self.directory=PeerDirectory();self.receiver=Receiver(id,accepted)
        qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        self.pub=self.create_publisher(String,"/autonomymind_agents/events",qos)
        self.sub=self.create_subscription(String,"/autonomymind_agents/events",self.receive,qos)
        self.timer=self.create_timer(.5,self.heartbeat)
    def publish(self,m):
        self.pub.publish(String(data=encode(m)))
    def heartbeat(self):
        self.publish(envelope(self.id,"*","heartbeat",{"capabilities":self.caps,"role":self.role},time.time(),3))
    def receive(self,raw):
        try:
            m=decode(raw.data);now=time.time()
            if m["kind"]=="heartbeat":
                self.directory.observe(m,now,time.monotonic());return
            status=self.receiver.check(m,now)
            if status=="other_recipient":return
            self.get_logger().info(f"{m['kind']} {status} correlation={m['correlation']}")
            if status!="accepted":return
            if self.role=="mission":
                peers=self.directory.discover("allocate",time.monotonic())
                if not peers:raise RuntimeError("no live coordinator")
                self.publish(envelope(self.id,peers[0],"mission.request",m["payload"],now,15,m["correlation"]))
            elif self.role=="coordination":
                peers=self.directory.discover("audit",time.monotonic())
                if not peers:raise RuntimeError("no live evaluator")
                report=run(m["payload"],ranker=self.ranker)
                self.publish(envelope(self.id,peers[0],"mission.report",report,time.time(),15,m["correlation"]))
            else:
                if m["payload"].get("motion_authorized") is not False:raise ValueError("not offline report")
                self.output.parent.mkdir(parents=True,exist_ok=True)
                name=self.output.with_suffix(".tmp")
                name.write_text(json.dumps({"correlation":m["correlation"],"report":m["payload"]},indent=2,allow_nan=False))
                name.replace(self.output)
                self.publish(envelope(self.id,"*","mission.result",{"metrics":m["payload"]["metrics"]},time.time(),15,m["correlation"]))
        except Exception as exc:
            self.get_logger().error(f"event rejected: {type(exc).__name__}: {exc}")

def main():
    p=argparse.ArgumentParser();p.add_argument("--id",required=True);p.add_argument("--role",choices=ROLES,required=True);p.add_argument("--output",default="/outbox/dds_pilot.json");p.add_argument("--model-config");a=p.parse_args()
    if not a.id.replace("_","").isalnum():raise ValueError("invalid node ID")
    ranker=None
    if a.model_config:
        if a.role!="coordination":raise ValueError("model configuration requires coordination role")
        from .onnx_ranker import load_ranker
        ranker=load_ranker(json.loads(Path(a.model_config).read_text()))
    rclpy.init();n=Worker(a.id,a.role,a.output,ranker)
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
