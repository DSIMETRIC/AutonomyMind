"""DDS integration test for shared evidence; all observations are synthetic."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String
from .dds_protocol import envelope,encode,decode,PeerDirectory

def main():
    rclpy.init();n=Node("shared_world_probe");directory=PeerDirectory();replies={}
    qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE)
    pub=n.create_publisher(String,"/autonomymind_agents/events",qos)
    def receive(raw):
        try:m=decode(raw.data)
        except ValueError:return
        if m["kind"]=="heartbeat":directory.observe(m,time.time(),time.monotonic())
        elif m["kind"] in ("world.receipt","world.snapshot") and time.time()<m["expires"]:replies[m["correlation"]]=m
    sub=n.create_subscription(String,"/autonomymind_agents/events",receive,qos)
    def wait(fn,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.05)
            if fn():return True
        return False
    def exchange(sender,recipient,kind,payload):
        m=envelope(sender,recipient,kind,payload,time.time(),3)
        pub.publish(String(data=encode(m)))
        if not wait(lambda:m["correlation"] in replies,2.5):raise RuntimeError("no world response")
        return replies.pop(m["correlation"])["payload"]
    try:
        if not wait(lambda:directory.discover("shared_world",time.monotonic()),30):raise RuntimeError("world agent missing")
        world=directory.discover("shared_world",time.monotonic())[0]
        for sender in ("synthetic_drone","synthetic_rover","synthetic_road"):
            p={"observation_id":"track-1","timestamp_s":time.time(),"frame_id":"world_enu","position_m":[1,2,0],"velocity_mps":[0,0,0],"confidence":.8,"uncertainty_m":.2,"label":"obstacle"}
            if exchange(sender,"*","world.observation",p)["status"]!="accepted":raise RuntimeError("observation rejected")
        scene={"kind":"unverified_semantic_description","metric_geometry_available":False,"motion_authorized":False,"requires_independent_validation":True,"image_sha256":"a"*64,"description":{"objects":["vehicle"],"hazards":[],"summary":"synthetic fixture"}}
        if exchange("synthetic_vision","*","scene.semantic",scene)["status"]!="accepted":raise RuntimeError("scene rejected")
        snap=exchange("probe",world,"world.query",{})
        sources={r["sender"] for r in snap["observations"]}
        if not {"synthetic_drone","synthetic_rover","synthetic_road"}<=sources or not any(r["sender"]=="synthetic_vision" for r in snap["semantic_reports"]) or snap["motion_authorized"] is not False:raise RuntimeError("incomplete snapshot")
        wait(lambda:False,6)
        snap=exchange("probe",world,"world.query",{})
        if any(r["sender"].startswith("synthetic_") for r in snap["observations"]+snap["semantic_reports"]):raise RuntimeError("expiry failed")
        print("PASS: shared world DDS ingestion, semantic separation, provenance and expiry")
    finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
