"""Read-only assessment of two Gazebo vehicles, not cooperative motion."""
import copy
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from autonomymind_interfaces.srv import AssessSimulationTrajectory as S

def main():
    rclpy.init();n=Node("fleet_rta_probe")
    latest={};subs=[];pubs={};clients={}
    for k in ("rover","drone","obstacle"):
        subs.append(n.create_subscription(Odometry,"/gazebo/"+k+"/odometry",lambda m,k=k:latest.__setitem__(k,m),qos_profile_sensor_data))
    for k in ("rover","drone"):
        pubs[k]=n.create_publisher(Odometry,"/fleet/"+k+"/simulation/odometry",qos_profile_sensor_data)
        clients[k]=n.create_client(S,"/fleet/"+k+"/simulation/assess_trajectory")
    def spin_until(fn,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.05)
            if fn():return True
        return False
    def pos(m):
        p=m.pose.pose.position;return (p.x,p.y,p.z)
    def assess(k,offset):
        own=copy.deepcopy(latest[k]);own.header.frame_id="world_enu"
        if not own.child_frame_id:own.child_frame_id=k
        # Flat reference simulation only; preserve raw contact height.
        raw_z=own.pose.pose.position.z
        if k=="rover" and -1e-4<=raw_z<0:own.pose.pose.position.z=0.
        q=S.Request();q.vehicle_id="sim-"+k;q.frame_id="world_enu"
        q.odom_stamp_sec=own.header.stamp.sec;q.odom_stamp_nanosec=own.header.stamp.nanosec
        q.sequence=time.time_ns();q.sample_dt_s=1.
        q.x_m=[0.,offset[0]];q.y_m=[0.,offset[1]];q.z_m=[0.,offset[2]]
        q.peak_speed_mps=math.sqrt(sum(x*x for x in offset));q.peak_vertical_speed_mps=abs(offset[2])
        q.environment_valid=True;q.communication_valid=True
        for other in ("rover","drone","obstacle"):
            if other==k:continue
            q.threat_position_m.extend([b-a for a,b in zip(pos(own),pos(latest[other]))])
            # Only stationary reference scene supported.
            q.threat_velocity_mps.extend([0.,0.,0.])
        pubs[k].publish(own)
        spin_until(lambda:False,.1)
        future=clients[k].call_async(q)
        if not spin_until(future.done,2.):raise RuntimeError("assessment timeout")
        a=future.result()
        return {"member":k,"offset":offset,"recorded":a.recorded,"reason":a.reason,"profile_id":a.profile_id,"raw_z":raw_z,"forwarded_z":own.pose.pose.position.z}
    try:
        if not spin_until(lambda:len(latest)==3 and all(c.service_is_ready() for c in clients.values()) and all(p.get_subscription_count()>0 for p in pubs.values()),40):raise RuntimeError("fleet inputs missing")
        for m in latest.values():
            v=m.twist.twist.linear
            if math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)>.05:raise RuntimeError("stationary scene required")
        records=[assess("rover",[0.,0.,0.]),assess("drone",[0.,0.,0.]),assess("rover",[2.,0.,0.]),assess("drone",[0.,0.,-3.])]
        out=Path("/gazebo/outbox/fleet_rta.json");out.parent.mkdir(parents=True,exist_ok=True)
        out.write_text(json.dumps(records,indent=2)+"\n")
        print(json.dumps(records,indent=2))
        if not records[0]["recorded"] or not records[1]["recorded"] or records[2]["recorded"] or records[3]["recorded"] or records[0]["profile_id"]==records[1]["profile_id"]:raise RuntimeError("unexpected assessment")
        print("PASS: independent rover and multirotor C++ reference assessments")
    finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
