"""DDS shared evidence service; read-only snapshots, no actuator interface."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String
from .dds_protocol import decode,encode,envelope,Receiver
from .shared_world import SharedWorld

class WorldAgent(Node):
    def __init__(self,id,frame,age,output):
        super().__init__("agent_"+id)
        self.id=id;self.world=SharedWorld(frame,age);self.output=Path(output)
        self.receiver=Receiver(id,{"world.query"})
        qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE)
        self.pub=self.create_publisher(String,"/autonomymind_agents/events",qos)
        self.sub=self.create_subscription(String,"/autonomymind_agents/events",self.receive,qos)
        self.timer=self.create_timer(.5,self.tick)
        self.get_logger().info("shared world evidence service started")
    def send(self,m):self.pub.publish(String(data=encode(m)))
    def tick(self):
        now=time.time();snapshot=self.world.snapshot(now)
        try:
            self.output.parent.mkdir(parents=True,exist_ok=True)
            tmp=self.output.with_suffix(".tmp")
            tmp.write_text(json.dumps(snapshot,indent=2,allow_nan=False)+"\n")
            tmp.replace(self.output)
            self.send(envelope(self.id,"*","heartbeat",{"capabilities":["shared_world"],"role":"world_model"},now,3))
        except Exception as exc:self.get_logger().error(f"snapshot failure: {exc}")
    def receive(self,raw):
        try:
            m=decode(raw.data);now=time.time()
            if m["kind"] in ("world.observation","world.member","scene.semantic") and m["recipient"]=="*":
                status=self.world.ingest(m,now)
                self.send(envelope(self.id,m["sender"],"world.receipt",{"status":status,"event_id":m["id"]},now,3,m["correlation"]))
            elif m["kind"]=="world.query" and self.receiver.check(m,now)=="accepted":
                self.send(envelope(self.id,m["sender"],"world.snapshot",self.world.snapshot(now),now,3,m["correlation"]))
        except Exception as exc:self.get_logger().warning(f"rejected world event: {type(exc).__name__}: {exc}")

def main():
    p=argparse.ArgumentParser();p.add_argument("--id",default="world");p.add_argument("--frame",default="world_enu");p.add_argument("--max-age",type=float,default=5);p.add_argument("--output",default="/outbox/shared_world.json");a=p.parse_args()
    rclpy.init();n=WorldAgent(a.id,a.frame,a.max_age,a.output)
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
