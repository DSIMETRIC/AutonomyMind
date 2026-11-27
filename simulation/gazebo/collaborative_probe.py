"""Check independent live sources and incremental occupied evidence over DDS."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String
from autonomymind_runtime.dds_protocol import envelope,encode,decode,PeerDirectory
from autonomymind_runtime.navigation import OccupancyGrid
from autonomymind_runtime.collaborative_evidence import occupied_sources

def main():
    rclpy.init();n=Node("collaborative_probe");directory=PeerDirectory();replies={}
    qos=QoSProfile(depth=128,reliability=ReliabilityPolicy.RELIABLE)
    pub=n.create_publisher(String,"/autonomymind_agents/events",qos)
    def receive(raw):
        try:m=decode(raw.data)
        except ValueError:return
        if m["kind"]=="heartbeat":directory.observe(m,time.time(),time.monotonic())
        elif m["kind"]=="world.snapshot" and m["recipient"]=="collaborative_probe" and time.time()<m["expires"]:replies[m["correlation"]]=m
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
        grid=OccupancyGrid(80,80,.25,(-10,-10))
        end=time.monotonic()+30
        while time.monotonic()<end:
            q=envelope("collaborative_probe",world,"world.query",{},time.time(),3)
            pub.publish(String(data=encode(q)))
            if not wait(lambda:q["correlation"] in replies,2):continue
            snap=replies.pop(q["correlation"])["payload"]
            cells=occupied_sources(snap,grid,time.time())
            a={c for c,s in cells.items() if "rover" in s};b={c for c,s in cells.items() if "drone" in s}
            if a and b and b-a and snap["motion_authorized"] is False:
                result={"source":"live_gazebo_stationary_reference","world_revision":snap["revision"],"rover_cells":len(a),"drone_cells":len(b),"shared_cells":len(a&b),"union_cells":len(a|b),"additional_drone_cells":len(b-a),"coverage_complete":False,"motion_authorized":False,"cells":[{"cell":list(c),"sources":sorted(s)} for c,s in sorted(cells.items())]}
                out=Path("/gazebo/outbox/collaborative_perception.json");out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+"\n")
                print(json.dumps({k:v for k,v in result.items() if k!="cells"},indent=2))
                print("PASS: independent Gazebo sensor producers contribute additional shared occupied evidence")
                return
            wait(lambda:False,.5)
        raise RuntimeError("two-source incremental evidence not demonstrated")
    finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
