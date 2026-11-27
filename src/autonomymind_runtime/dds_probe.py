"""Cross-process DDS integration probe. No commands to robots."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String
from .dds_protocol import envelope,encode,decode,PeerDirectory

def main():
    p=argparse.ArgumentParser();p.add_argument("--scenario",required=True);p.add_argument("--expect-no-coordinator",action="store_true");a=p.parse_args()
    rclpy.init();n=Node("dds_pilot_probe")
    directory=PeerDirectory();results={}
    qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE)
    pub=n.create_publisher(String,"/autonomymind_agents/events",qos)
    def receive(raw):
        try:m=decode(raw.data)
        except ValueError:return
        if m["kind"]=="heartbeat":directory.observe(m,time.time(),time.monotonic())
        elif m["kind"]=="mission.result" and m["recipient"]=="*" and time.time()<m["expires"]:results[m["correlation"]]=m["payload"]
    sub=n.create_subscription(String,"/autonomymind_agents/events",receive,qos)
    def wait(predicate,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.1)
            if predicate():return True
        return False
    try:
        if not wait(lambda:directory.discover("mission",time.monotonic()) and directory.discover("audit",time.monotonic()),30):
            raise RuntimeError("mission/evaluation not discovered")
        if a.expect_no_coordinator:
            # Observe for longer than the peer lease, then require no live coordinator.
            wait(lambda:False,5)
            if directory.discover("allocate",time.monotonic()):raise RuntimeError("coordinator still live")
            print("PASS: missing coordinator detected; no mission submitted");return
        if not wait(lambda:directory.discover("allocate",time.monotonic()),30):raise RuntimeError("coordinator not discovered")
        # Give all workers a heartbeat interval to populate their local directories.
        wait(lambda:False,2)
        m=envelope("operator",directory.discover("mission",time.monotonic())[0],"mission.submit",json.loads(Path(a.scenario).read_text()),time.time(),15)
        pub.publish(String(data=encode(m)))
        if not wait(lambda:m["correlation"] in results,20):raise RuntimeError("mission result timeout")
        metrics=results[m["correlation"]]["metrics"]
        print(json.dumps(metrics,indent=2))
        if metrics["tasks_completed"]!=metrics["tasks_total"] or metrics["tasks_reassigned_to_different_member"]!=1 or metrics["rejected_completions"]!=1:raise RuntimeError("unexpected pilot results")
        print("PASS: mission -> coordination -> evaluation over ROS 2 DDS")
    finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
