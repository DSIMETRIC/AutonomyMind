"""Offline image interpretation agent; publishes semantic evidence only via DDS."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String
from .dds_protocol import envelope,encode
from .local_reasoning import LocalModel,interpret_scene

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--config",required=True);p.add_argument("--image",required=True)
    p.add_argument("--output",required=True);p.add_argument("--id",default="vision")
    a=p.parse_args()
    c=json.loads(Path(a.config).read_text())
    client=LocalModel(c["url"],c["model"],c.get("timeout_s",60))
    result=interpret_scene(client,a.image)
    out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
    tmp=out.with_suffix(".tmp");tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n");tmp.replace(out)
    rclpy.init();node=Node("agent_"+a.id)
    pub=node.create_publisher(String,"/autonomymind_agents/events",QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE))
    try:
        until=time.monotonic()+5
        while pub.get_subscription_count()==0 and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
        if pub.get_subscription_count()==0:raise RuntimeError("no DDS subscribers; report saved locally")
        event=envelope(a.id,"*","scene.semantic",result,time.time(),10)
        pub.publish(String(data=encode(event)))
        if not pub.wait_for_all_acked(timeout=Duration(seconds=5)):raise RuntimeError("DDS acknowledgement timeout")
        print("Semantic report published; application consumption not verified")
    finally:node.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
