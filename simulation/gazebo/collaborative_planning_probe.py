"""Compare local and shared A* proposals from live DDS snapshots."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String
from autonomymind_runtime.dds_protocol import envelope,encode,decode,PeerDirectory
from autonomymind_runtime.navigation import OccupancyGrid
from autonomymind_runtime.collaborative_planning import compare_routes

def main():
    rclpy.init();n=Node("collaborative_planning_probe");directory=PeerDirectory();replies={}
    qos=QoSProfile(depth=128,reliability=ReliabilityPolicy.RELIABLE)
    pub=n.create_publisher(String,"/autonomymind_agents/events",qos)
    def receive(raw):
        try:m=decode(raw.data)
        except ValueError:return
        if m["kind"]=="heartbeat":directory.observe(m,time.time(),time.monotonic())
        elif m["kind"]=="world.snapshot" and m["recipient"]=="collaborative_planning_probe" and time.time()<m["expires"]:replies[m["correlation"]]=m
    sub=n.create_subscription(String,"/autonomymind_agents/events",receive,qos)
    def wait(fn,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.05)
            if fn():return True
        return False
    try:
        if not wait(lambda:directory.discover("shared_world",time.monotonic()),90):raise RuntimeError("world unavailable")
        world=directory.discover("shared_world",time.monotonic())[0]
        config=json.loads(Path("/gazebo/collaborative_planning_scenario.json").read_text())
        end=time.monotonic()+30
        while time.monotonic()<end:
            q=envelope("collaborative_planning_probe",world,"world.query",{},time.time(),3)
            pub.publish(String(data=encode(q)))
            if not wait(lambda:q["correlation"] in replies,2):continue
            snap=replies.pop(q["correlation"])["payload"]
            result=compare_routes(snap,time.time(),config)
            result["source"]="live_gazebo_stationary_reference"
            if result["local_cells"] and result["remote_additional_cells"]:
                out=Path("/gazebo/outbox/collaborative_planning.json");out.parent.mkdir(parents=True,exist_ok=True)
                out.write_text(json.dumps(result,indent=2)+"\n")
                if result["status"]=="alternate_route":
                    print(json.dumps({k:v for k,v in result.items() if k not in ("local_route","shared_route","invalidated_local_cells")},indent=2))
                    print("PASS: remote sensor evidence invalidates local route and produces alternate A* route")
                    return
            wait(lambda:False,.5)
        raise RuntimeError("remote-evidence route change not demonstrated; inspect report")
    finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
