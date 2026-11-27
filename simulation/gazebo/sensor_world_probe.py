"""Read-only live Gazebo scan -> DDS world -> A* -> C++ assessment experiment."""
import copy
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from autonomymind_interfaces.srv import AssessSimulationTrajectory as S
from autonomymind_runtime.dds_protocol import envelope,encode,decode,PeerDirectory
from autonomymind_runtime.navigation import OccupancyGrid
from autonomymind_runtime.sensor_evidence import project_scan,route_from_hits

def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9

def main():
    rclpy.init();n=Node("sensor_world_probe")
    latest={};replies={};directory=PeerDirectory();subs=[]
    def sensor(k,m):latest[k]=(m,time.monotonic())
    subs.append(n.create_subscription(Odometry,"/gazebo/rover/odometry",lambda m:sensor("odom",m),qos_profile_sensor_data))
    subs.append(n.create_subscription(LaserScan,"/mobile/scan",lambda m:sensor("scan",m),qos_profile_sensor_data))
    qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE)
    pub=n.create_publisher(String,"/autonomymind_agents/events",qos)
    def receive(raw):
        try:m=decode(raw.data)
        except ValueError:return
        if m["kind"]=="heartbeat":directory.observe(m,time.time(),time.monotonic())
        elif m["recipient"]=="gazebo_sensor" and m["kind"] in ("world.receipt","world.snapshot") and time.time()<m["expires"]:replies[m["correlation"]]=m
    subs.append(n.create_subscription(String,"/autonomymind_agents/events",receive,qos))
    odom_pub=n.create_publisher(Odometry,"/autonomymind_sim/simulation/odometry",qos_profile_sensor_data)
    client=n.create_client(S,"/autonomymind_sim/simulation/assess_trajectory")
    def wait(fn,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.02)
            if fn():return True
        return False
    def ready():
        return len(latest)==2 and all(time.monotonic()-v[1]<.6 for v in latest.values()) and abs(stamp(latest["odom"][0])-stamp(latest["scan"][0]))<.35
    def pose(m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        if m.header.frame_id!="world_enu" or abs(q.x)>.05 or abs(q.y)>.05 or abs(p.z)>.09:raise RuntimeError("flat reference pose required")
        v=m.twist.twist.linear;w=m.twist.twist.angular
        if math.hypot(v.x,v.y)>.05 or abs(v.z)>.05 or abs(w.z)>.05:raise RuntimeError("stationary reference required")
        return (p.x,p.y,p.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    def exchange(recipient,kind,payload):
        m=envelope("gazebo_sensor",recipient,kind,payload,time.time(),5)
        pub.publish(String(data=encode(m)))
        if not wait(lambda:m["correlation"] in replies,2):raise RuntimeError("world response missing")
        return replies.pop(m["correlation"])["payload"]
    try:
        if not wait(lambda:ready() and directory.discover("shared_world",time.monotonic()) and client.service_is_ready() and odom_pub.get_subscription_count()>0,90):raise RuntimeError("missing streams or services")
        world=directory.discover("shared_world",time.monotonic())[0]
        ego=copy.deepcopy(latest["odom"][0]);scan=latest["scan"][0];p=pose(ego)
        hits=project_scan(scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max,p)
        if not hits:raise RuntimeError("no finite obstacle returns")
        # Batch publication avoids per-beam round trips; retain exact capture timestamps.
        now=time.time();ids=set()
        for i,point in hits:
            payload={"observation_id":"beam-"+str(i),"timestamp_s":now,"frame_id":"world_enu","position_m":point,"velocity_mps":[0.,0.,0.],"confidence":1.,"uncertainty_m":.15,"label":"lidar_surface_return","velocity_known":False,"simulation_stamp_s":stamp(scan),"source":"gazebo_gpu_lidar"}
            m=envelope("gazebo_sensor","*","world.observation",payload,now,5);ids.add(m["id"]);pub.publish(String(data=encode(m)))
        if not wait(lambda:sum(1 for m in replies.values() if m["payload"].get("event_id") in ids and m["payload"].get("status")=="accepted")==len(ids),3):raise RuntimeError("world ingestion incomplete")
        snap=exchange(world,"world.query",{})
        points=[r["payload"]["position_m"] for r in snap["observations"] if r["event_id"] in ids]
        if len(points)!=len(hits):raise RuntimeError("snapshot incomplete")
        config=json.loads(Path("/gazebo/sensor_world_scenario.json").read_text())
        grid=OccupancyGrid(**config["map"]);grid.free={(x,y) for x in range(grid.width) for y in range(grid.height)}
        goal=config["goal"];clearance=config["clearance_m"]
        before=route_from_hits(grid,p[:2],goal,[],clearance)
        after=route_from_hits(grid,p[:2],goal,points,clearance)
        if not before or before==after or not after:raise RuntimeError("scan did not produce alternate route")
        if not ready():raise RuntimeError("stale live streams")
        current=copy.deepcopy(latest["odom"][0]);cp=pose(current)
        if math.dist(p[:3],cp[:3])>.03:raise RuntimeError("ego moved during experiment")
        if -1e-4<=cp[2]<0:current.pose.pose.position.z=0.
        # Evaluate just the first route segment, not the entire A* route.
        target=after[1];dx=target[0]-cp[0];dy=target[1]-cp[1]
        q=S.Request();q.vehicle_id="sim-rover";q.frame_id="world_enu";q.sequence=time.time_ns()
        q.odom_stamp_sec=current.header.stamp.sec;q.odom_stamp_nanosec=current.header.stamp.nanosec
        q.sample_dt_s=2.;q.x_m=[0.,dx];q.y_m=[0.,dy];q.z_m=[0.,0.]
        q.peak_speed_mps=math.hypot(dx,dy)/2
        q.peak_accel_mps2=q.peak_speed_mps;q.peak_jerk_mps3=q.peak_speed_mps
        for point in points[:100]:
            q.threat_position_m.extend([point[0]-cp[0],point[1]-cp[1],0.])
            q.threat_velocity_mps.extend([0.,0.,0.])
        # Static scene assumption, not validated environment coverage.
        q.environment_valid=True;q.communication_valid=True
        odom_pub.publish(current);wait(lambda:False,.1)
        future=client.call_async(q)
        if not wait(future.done,2) or future.result() is None:raise RuntimeError("RTA timeout")
        a=future.result()
        result={"source":"live_gazebo_lidar_reference","world_revision":snap["revision"],"observations":len(points),"route_before":before,"route_after":after,"rta_recorded":a.recorded,"rta_reason":a.reason,"profile_id":a.profile_id,"motion_authorized":False,"coverage_complete":False}
        out=Path("/gazebo/outbox/sensor_world.json");out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+"\n")
        print(json.dumps({k:v for k,v in result.items() if not k.startswith("route_")},indent=2))
        if a.reason not in ("reference_checks_passed","constraint_rejected"):raise RuntimeError("RTA did not evaluate candidate")
        print("PASS: live Gazebo scan -> DDS world -> changed A* route -> C++ assessment")
    finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
